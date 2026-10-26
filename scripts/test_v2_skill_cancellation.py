"""Real ROS fault injection: delayed cancellation and late goal acceptance."""
import tempfile
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import ExecuteTrajectory
from qda_interfaces.action import RobotSkill
from qda_brain.skill_server import SkillServer
from qda_brain.contracts import SCENE_VERSION


class Driver(Node):
    def __init__(self):
        super().__init__('delayed_trajectory_driver');self.group=ReentrantCallbackGroup();self.delay_accept=False;self.stopped=False
        self.server=ActionServer(self,ExecuteTrajectory,'/execute_trajectory',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
    def accept(self,goal):
        if self.delay_accept:time.sleep(.6)
        return GoalResponse.ACCEPT
    def execute(self,handle):
        end=time.monotonic()+4
        while not handle.is_cancel_requested and time.monotonic()<end:time.sleep(.01)
        time.sleep(.3);self.stopped=True
        if handle.is_cancel_requested:handle.canceled()
        else:handle.abort()
        return ExecuteTrajectory.Result()


def main():
    temp=tempfile.TemporaryDirectory();rclpy.init(args=['--ros-args','-p','journal_path:='+temp.name+'/tasks.sqlite'])
    skills=SkillServer();driver=Driver();executor=MultiThreadedExecutor(num_threads=8)
    for node in (skills,driver):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert skills.move.wait_for_server(timeout_sec=5)
        timer=threading.Timer(.5,lambda:setattr(skills,'stop',True));timer.start()
        try:skills.action(skills.move,ExecuteTrajectory.Goal(),time.monotonic()+5);raise AssertionError('Cancellation ignored')
        except RuntimeError as exc:assert str(exc)=='CANCELLED',exc
        timer.join();assert driver.stopped and skills.child is None and not skills.quarantine
        driver.delay_accept=True;driver.stopped=False;skills.stop=False
        try:skills.action(skills.move,ExecuteTrajectory.Goal(),time.monotonic()+.15);raise AssertionError('Acceptance deadline ignored')
        except TimeoutError:pass
        assert skills.quarantine=='LATE_CHILD_ACCEPTANCE_UNCONFIRMED'
        end=time.monotonic()+4
        while not driver.stopped and time.monotonic()<end:time.sleep(.02)
        assert driver.stopped,'Late child was not cancelled'
        goal=RobotSkill.Goal(task_id='new-after-late',scene_version=SCENE_VERSION,skill='inspect',object_id='none',target_id='none',arm='none')
        assert skills.accept(goal)==GoalResponse.REJECT
        print('PASS: cancellation waits for child terminal; late acceptance cancels and quarantines subsequent skills')
    finally:
        executor.shutdown();thread.join(timeout=2);skills.destroy_node();driver.destroy_node();rclpy.shutdown();temp.cleanup()


if __name__=='__main__':main()
