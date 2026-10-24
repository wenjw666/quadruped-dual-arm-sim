#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ignition/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "qda_interfaces/srv/attach.hpp"
#include "qda_interfaces/srv/detach.hpp"

namespace qda
{

struct Job
{
  bool attach{true};
  std::string parent_model;
  std::string parent_link;
  std::string child_model;
  std::string child_link;
  std::shared_ptr<std::promise<std::pair<bool, std::string>>> done;
};

class GraspPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    attach_srv_ = node_->create_service<qda_interfaces::srv::Attach>(
      "/qda_grasp/attach",
      [this](const std::shared_ptr<qda_interfaces::srv::Attach::Request> req,
             std::shared_ptr<qda_interfaces::srv::Attach::Response> res) {
        this->HandleAttach(req, res);
      });
    detach_srv_ = node_->create_service<qda_interfaces::srv::Detach>(
      "/qda_grasp/detach",
      [this](const std::shared_ptr<qda_interfaces::srv::Detach::Request> req,
             std::shared_ptr<qda_interfaces::srv::Detach::Response> res) {
        this->HandleDetach(req, res);
      });
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_vx_.store(msg->linear.x);
        cmd_vy_.store(msg->linear.y);
        cmd_wz_.store(msg->angular.z);
        command_pending_ = true;
      });
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 200);
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    if (sdf->HasElement("pelvis_z")) {
      pelvis_z_ = sdf->Get<double>("pelvis_z");
    }
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) { this->OnUpdate(); });
    RCLCPP_INFO(node_->get_logger(), "抓取焊接插件已加载，底盘高度 %.3f", pelvis_z_);
  }

private:
  void HandleAttach(
    const std::shared_ptr<qda_interfaces::srv::Attach::Request> req,
    std::shared_ptr<qda_interfaces::srv::Attach::Response> res)
  {
    Job job;
    job.attach = true;
    job.parent_model = req->parent_model;
    job.parent_link = req->parent_link;
    job.child_model = req->child_model;
    job.child_link = req->child_link;
    job.done = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto future = job.done->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      jobs_.push_back(std::move(job));
    }
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      res->success = false;
      res->message = "等待物理步超时";
      return;
    }
    const auto result = future.get();
    res->success = result.first;
    res->message = result.second;
  }

  void HandleDetach(
    const std::shared_ptr<qda_interfaces::srv::Detach::Request> req,
    std::shared_ptr<qda_interfaces::srv::Detach::Response> res)
  {
    Job job;
    job.attach = false;
    job.child_model = req->child_model;
    job.done = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto future = job.done->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      jobs_.push_back(std::move(job));
    }
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      res->success = false;
      res->message = "等待物理步超时";
      return;
    }
    const auto result = future.get();
    res->success = result.first;
    res->message = result.second;
  }

  void HoldStill()
  {
    auto model = world_->ModelByName("qda_robot");
    if (!model || released_) {
      return;
    }
    const double now = world_->SimTime().Double();
    if (!holding_) {
      Job job;
      job.parent_model = "spawn_anchor";
      job.parent_link = "link";
      job.child_model = "qda_robot";
      job.child_link = "base_link";
      const auto welded = AttachNow(job);
      if (!welded.first) {
        return;
      }
      holding_ = true;
      hold_until_ = now + 1.0;
      RCLCPP_INFO(node_->get_logger(), "底盘已焊在出生点，等腿关节锁住");
    }
    if (now < hold_until_) {
      return;
    }
    DetachNow("qda_robot");
    const auto pose = model->WorldPose();
    track_x_ = pose.Pos().X();
    track_y_ = pose.Pos().Y();
    track_yaw_ = pose.Rot().Yaw();
    last_track_time_ = now;
    model->SetLinearVel(ignition::math::Vector3d::Zero);
    model->SetAngularVel(ignition::math::Vector3d::Zero);
    released_ = true;
    RCLCPP_INFO(node_->get_logger(), "底盘已松开，改为按速度指令贴地移动");
  }

  void TrackBase()
  {
    if (!released_) {
      return;
    }
    auto model = world_->ModelByName("qda_robot");
    if (!model) {
      return;
    }
    const double now = world_->SimTime().Double();
    double dt = now - last_track_time_;
    last_track_time_ = now;
    if (dt < 0.0 || dt > 0.05) {
      dt = 0.0;
    }
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (command_pending_) {
        last_command_time_ = now;
        command_pending_ = false;
      }
      vx = cmd_vx_.load();
      vy = cmd_vy_.load();
      wz = cmd_wz_.load();
    }
    auto bounded = [](double value, double lo, double hi) {
      return std::isfinite(value) ? std::max(lo, std::min(hi, value)) : 0.0;
    };
    // A dead publisher must not leave the kinematic base moving indefinitely.
    if (now - last_command_time_ > 0.5 || now < last_command_time_) {
      vx = vy = wz = 0.0;
    }
    auto slew = [&](double current, double target, double acceleration) {
      return current + bounded(target - current, -acceleration * dt, acceleration * dt);
    };
    applied_vx_ = slew(applied_vx_, bounded(vx, -0.05, 0.22), 0.35);
    applied_vy_ = slew(applied_vy_, bounded(vy, -0.12, 0.12), 0.25);
    applied_wz_ = slew(applied_wz_, bounded(wz, -0.7, 0.7), 0.8);
    vx = applied_vx_;
    vy = applied_vy_;
    wz = applied_wz_;
    const double c = std::cos(track_yaw_ + 0.5 * wz * dt);
    const double s = std::sin(track_yaw_ + 0.5 * wz * dt);
    track_x_ += (c * vx - s * vy) * dt;
    track_y_ += (s * vx + c * vy) * dt;
    track_yaw_ += wz * dt;
    model->SetWorldPose(ignition::math::Pose3d(track_x_, track_y_, pelvis_z_, 0.0, 0.0, track_yaw_));
    // Velocity stays zero so physics does not integrate a second copy of the motion.
    model->SetLinearVel(ignition::math::Vector3d::Zero);
    model->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  void PublishOdometry()
  {
    const double now = world_->SimTime().Double();
    if (now - last_odom_time_ < 0.02) return;
    auto model = world_->ModelByName("qda_robot");
    if (!model) return;
    last_odom_time_ = now;
    const auto pose = model->WorldPose();
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = world_->SimTime().sec;
    msg.header.stamp.nanosec = world_->SimTime().nsec;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();
    msg.pose.pose.position.z = pose.Pos().Z();
    msg.pose.pose.orientation.x = pose.Rot().X();
    msg.pose.pose.orientation.y = pose.Rot().Y();
    msg.pose.pose.orientation.z = pose.Rot().Z();
    msg.pose.pose.orientation.w = pose.Rot().W();
    // P3D sees zero physics velocity because TrackBase applies the pose directly.
    msg.twist.twist.linear.x = applied_vx_;
    msg.twist.twist.linear.y = applied_vy_;
    msg.twist.twist.angular.z = applied_wz_;
    odom_pub_->publish(msg);
  }

  void PublishKinematicImu()
  {
    // SetWorldPose hides the motion from the Gazebo IMU, and FAST-LIVO then
    // paints each scan further along a false trajectory. Report the specific
    // force of the pose we actually applied.
    const double now = world_->SimTime().Double();
    if (!imu_started_) {
      imu_started_ = true;
      last_imu_time_ = now;
      last_imu_x_ = track_x_;
      last_imu_y_ = track_y_;
      last_imu_yaw_ = track_yaw_;
      return;
    }
    const double dt = now - last_imu_time_;
    if (dt < 0.005) {
      return;
    }
    const double dx = track_x_ - last_imu_x_;
    const double dy = track_y_ - last_imu_y_;
    double dyaw = track_yaw_ - last_imu_yaw_;
    constexpr double kPi = 3.141592653589793;
    while (dyaw > kPi) dyaw -= 2.0 * kPi;
    while (dyaw < -kPi) dyaw += 2.0 * kPi;
    const double mid = last_imu_yaw_ + 0.5 * dyaw;
    const double c = std::cos(mid);
    const double s = std::sin(mid);
    auto limit = [](double value, double bound) {
      if (value > bound) return bound;
      if (value < -bound) return -bound;
      return value;
    };
    const double vx = limit((c * dx + s * dy) / dt, 1.0);
    const double vy = limit((-s * dx + c * dy) / dt, 1.0);
    const double wz = limit(dyaw / dt, 2.0);
    // Differentiate the bounded commanded velocity, not jittery pose samples.
    const double ax = (applied_vx_ - last_imu_vx_) / dt - applied_wz_ * applied_vy_;
    const double ay = (applied_vy_ - last_imu_vy_) / dt + applied_wz_ * applied_vx_;
    last_imu_vx_ = applied_vx_;
    last_imu_vy_ = applied_vy_;
    sensor_msgs::msg::Imu msg;
    const int64_t ns = static_cast<int64_t>(now * 1.0e9);
    msg.header.stamp.sec = static_cast<int32_t>(ns / 1000000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000);
    msg.header.frame_id = "imu_link";
    msg.orientation.z = std::sin(0.5 * track_yaw_);
    msg.orientation.w = std::cos(0.5 * track_yaw_);
    msg.angular_velocity.z = wz;
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = 9.81;
    msg.orientation_covariance[0] = 0.01;
    msg.orientation_covariance[4] = 0.01;
    msg.orientation_covariance[8] = 0.01;
    msg.angular_velocity_covariance[0] = 1.0e-4;
    msg.angular_velocity_covariance[4] = 1.0e-4;
    msg.angular_velocity_covariance[8] = 1.0e-4;
    msg.linear_acceleration_covariance[0] = 1.0e-3;
    msg.linear_acceleration_covariance[4] = 1.0e-3;
    msg.linear_acceleration_covariance[8] = 1.0e-3;
    imu_pub_->publish(msg);
    last_imu_time_ = now;
    last_imu_x_ = track_x_;
    last_imu_y_ = track_y_;
    last_imu_yaw_ = track_yaw_;
  }

  void OnUpdate()
  {
    HoldStill();
    TrackBase();
    PublishOdometry();
    PublishKinematicImu();
    std::vector<Job> jobs;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      jobs.swap(jobs_);
    }
    for (auto & job : jobs) {
      if (job.attach) {
        job.done->set_value(AttachNow(job));
      } else {
        job.done->set_value(DetachNow(job.child_model));
      }
    }
  }

  std::pair<bool, std::string> AttachNow(const Job & job)
  {
    auto parent = world_->ModelByName(job.parent_model);
    auto child = world_->ModelByName(job.child_model);
    if (!parent || !child) {
      return {false, "模型不存在"};
    }
    auto parent_link = parent->GetLink(job.parent_link);
    auto child_link = child->GetLink(job.child_link);
    if (!parent_link || !child_link) {
      return {false, "连杆不存在"};
    }
    DetachNow(job.child_model);
    const std::string joint_name = "qda_grasp_" + job.child_model;
    auto joint = world_->Physics()->CreateJoint("fixed", parent);
    if (!joint) {
      return {false, "无法创建固定关节"};
    }
    joint->SetName(joint_name);
    const auto relative = parent_link->WorldPose().Inverse() * child_link->WorldPose();
    joint->Attach(parent_link, child_link);
    joint->Load(parent_link, child_link, relative);
    joint->Init();
    grasps_[job.child_model] = joint;
    return {true, "已吸附 " + job.child_model};
  }

  std::pair<bool, std::string> DetachNow(const std::string & child_model)
  {
    auto found = grasps_.find(child_model);
    if (found == grasps_.end()) {
      return {true, "本来就没有吸附"};
    }
    auto joint = found->second;
    if (joint) {
      auto child = joint->GetChild();
      joint->Detach();
      joint->Fini();
      if (child) {
        child->SetLinearVel(ignition::math::Vector3d::Zero);
        child->SetAngularVel(ignition::math::Vector3d::Zero);
      }
    }
    grasps_.erase(found);
    return {true, "已松开 " + child_model};
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<qda_interfaces::srv::Attach>::SharedPtr attach_srv_;
  rclcpp::Service<qda_interfaces::srv::Detach>::SharedPtr detach_srv_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::mutex cmd_mutex_;
  bool command_pending_{false};
  double last_command_time_{-1.0};
  double last_odom_time_{0.0};
  double applied_vx_{0.0}, applied_vy_{0.0}, applied_wz_{0.0};
  double last_imu_vx_{0.0}, last_imu_vy_{0.0};
  std::atomic<double> cmd_vx_{0.0};
  std::atomic<double> cmd_vy_{0.0};
  std::atomic<double> cmd_wz_{0.0};
  double pelvis_z_{0.56};
  double track_x_{0.0};
  double track_y_{0.0};
  double track_yaw_{0.0};
  double last_track_time_{0.0};
  gazebo::event::ConnectionPtr update_;
  std::mutex mutex_;
  std::vector<Job> jobs_;
  std::unordered_map<std::string, gazebo::physics::JointPtr> grasps_;
  bool holding_{false};
  bool released_{false};
  bool imu_started_{false};
  double hold_until_{0.0};
  double last_imu_time_{0.0};
  double last_imu_x_{0.0};
  double last_imu_y_{0.0};
  double last_imu_yaw_{0.0};
};

}  // namespace qda

GZ_REGISTER_WORLD_PLUGIN(qda::GraspPlugin)
