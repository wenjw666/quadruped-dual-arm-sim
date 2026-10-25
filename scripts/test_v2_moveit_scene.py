"""Plan against the live world-fixed OctoMap, then execute through MoveIt/PD."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.action import ExecuteTrajectory


def main():
    rclpy.init();node=rclpy.create_node('v2_moveit_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};stable=[];active=[];report={'passed':False};started=time.monotonic()
    def state(msg):
        data=json.loads(msg.data);latest['status']=data
        velocity=data['actual_body_velocity']
        stable.append(math.hypot(*velocity[:2])<.03 and abs(velocity[2])<.08 and data['tilt_rad']<.10)
        del stable[:-5]
    node.create_subscription(String,'/locomotion/rl_status',state,10)
    node.create_subscription(JointState,'/joint_states',lambda msg:latest.update(joints=msg),10)
    def wait(predicate,seconds=60.):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if latest.get('status',{}).get('fault'):raise AssertionError(latest['status']['fault'])
        if not predicate():raise TimeoutError('MoveIt integration timeout')
    def result(future,seconds=60.):
        wait(future.done,seconds);return future.result()
    hold=node.create_client(SetBool,'/motion/hold')
    plan=node.create_client(GetMotionPlan,'/plan_kinematic_path')
    execute=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
    try:
        wait(lambda:hold.service_is_ready() and plan.service_is_ready() and execute.server_is_ready())
        assert result(hold.call_async(SetBool.Request(data=True))).success
        wait(lambda:len(stable)==5 and all(stable) and 'joints' in latest)
        report['base_settled']=True
        report['plans']=[]
        for side in ('left','right'):
            joints=dict(zip(latest['joints'].name,latest['joints'].position))
            names=[f'{side}_joint_link{i}' for i in range(1,8)];home=[joints[n] for n in names]
            target=home.copy();target[0]+=.05 if side=='left' else -.05
            for positions in (target,home):
                request=GetMotionPlan.Request();motion=request.motion_plan_request
                motion.group_name=side+'_arm';motion.planner_id='RRTConnect';motion.pipeline_id='ompl'
                motion.num_planning_attempts=1;motion.allowed_planning_time=5.
                motion.max_velocity_scaling_factor=.15;motion.max_acceleration_scaling_factor=.15
                motion.start_state.is_diff=True
                motion.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=float(v),
                    tolerance_above=.01,tolerance_below=.01,weight=1.) for n,v in zip(names,positions)])]
                answer=result(plan.call_async(request)).motion_plan_response
                row={'arm':side,'planning_error_code':answer.error_code.val,'points':len(answer.trajectory.joint_trajectory.points)}
                report['plans'].append(row);assert answer.error_code.val==1 and row['points']>0,row
                goal=ExecuteTrajectory.Goal(trajectory=answer.trajectory)
                handle=result(execute.send_goal_async(goal));assert handle.accepted;active.append(handle)
                execution=result(handle.get_result_async(),90);active.remove(handle)
                row.update(action_status=execution.status,execution_error_code=execution.result.error_code.val)
                print(json.dumps(row),flush=True)
                assert execution.status==4 and execution.result.error_code.val==1,row
        report['passed']=True
    except Exception as exc:
        report['failure']=f'{type(exc).__name__}: {exc}';raise
    finally:
        for handle in active:handle.cancel_goal_async()
        report['wall_seconds']=time.monotonic()-started
        Path('/ws/output/v2_scene/moveit_integration.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
