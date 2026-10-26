"""Isolated real ROS Actions with a controlled skill driver; no physics claims."""
import json
import tempfile
import threading
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from qda_interfaces.action import RobotSkill
from qda_interfaces.srv import BrainRequest
from qda_brain.mission_node import MissionCoordinator
from qda_brain.contracts import SCENE_VERSION,parse_local


class Driver(Node):
    def __init__(self):
        super().__init__('controlled_skill_driver');self.group=ReentrantCallbackGroup();self.calls=[];self.blocking=False
        self.action=ActionServer(self,RobotSkill,'/robot_skill',execute_callback=self.execute,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_service(BrainRequest,'/skills/query',self.query,callback_group=self.group)
    def query(self,request,response):
        data=json.loads(request.request_json);response.response_json=json.dumps(dict(ok=True,token='controlled-session'));return response
    def execute(self,h):
        self.calls.append((h.request.skill,h.request.arm));self.blocking=h.request.skill=='visual_align'
        if self.blocking:
            end=time.monotonic()+10
            while not h.is_cancel_requested and time.monotonic()<end:time.sleep(.01)
            time.sleep(.2) # acknowledgement intentionally follows request
            h.canceled();self.blocking=False
            return RobotSkill.Result(success=False,error_code='CANCELLED',report_json='{"success":false}')
        time.sleep(.1);failed=h.request.arm=='right'
        if failed:h.abort()
        else:h.succeed()
        return RobotSkill.Result(success=not failed,error_code='CONTROLLED_FAILURE' if failed else '',report_json=json.dumps({'success':not failed}))


def main():
    directory=tempfile.TemporaryDirectory()
    rclpy.init(args=['--ros-args','-p','journal_path:='+directory.name+'/missions.sqlite'])
    coordinator=MissionCoordinator();driver=Driver();probe=Node('mission_protocol_probe')
    client=probe.create_client(BrainRequest,'/mission/request');executor=MultiThreadedExecutor(num_threads=8)
    for node in (coordinator,driver,probe):executor.add_node(node)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    def call(data):
        assert client.wait_for_service(timeout_sec=3)
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(data)));end=time.monotonic()+4
        while not f.done() and time.monotonic()<end:time.sleep(.01)
        assert f.done();return json.loads(f.result().response_json)
    def submit(mid,texts):return call(dict(op='mission_submit',mission_id=mid,plan=dict(scene_version=SCENE_VERSION,steps=[parse_local(t) for t in texts])))
    def terminal(mid):
        end=time.monotonic()+8
        while time.monotonic()<end:
            rows=call(dict(op='mission_status',mission_id=mid))['missions']
            if rows and rows[0]['status'] not in ('RESERVED','RUNNING'):return rows[0]
            time.sleep(.02)
        raise TimeoutError(mid)
    report={'source':'isolated ROS protocol with controlled skill driver'}
    try:
        assert submit('order',['查询状态','左臂回位','查询状态'])['accepted']
        report['sequence']=terminal('order');assert report['sequence']['status']=='SUCCEEDED'
        assert call({'op':'mission_cancel','mission_id':'order'})['ok']
        assert not coordinator.cancelled.is_set(),'Cancelling a terminal mission affected unrelated future work'
        assert driver.calls==[('inspect','none'),('home','left'),('inspect','none')]
        before=len(driver.calls);assert submit('order',['查询状态','左臂回位','查询状态'])['duplicate']
        time.sleep(.15);assert len(driver.calls)==before
        assert not submit('order',['查询状态'])['ok']
        assert submit('failure',['查询状态','右臂回位','查询状态'])['accepted']
        report['failure']=terminal('failure');assert report['failure']['status']=='FAILED'
        assert len(driver.calls)==before+2
        assert submit('cancel',['左臂对准','查询状态'])['accepted']
        end=time.monotonic()+3
        while not driver.blocking and time.monotonic()<end:time.sleep(.01)
        assert driver.blocking;assert not submit('busy',['查询状态'])['ok']
        assert not call({'op':'mission_cancel','mission_id':'wrong'})['ok']
        assert call({'op':'mission_cancel','mission_id':'cancel'})['ok']
        report['cancel']=terminal('cancel');assert report['cancel']['status']=='CANCELLED'
        assert report['cancel']['result']['completed_steps']==0
        assert not driver.blocking and coordinator.child is None and not coordinator.quarantine
        count=len(driver.calls);assert submit('cancel',['左臂对准','查询状态'])['duplicate']
        time.sleep(.1);assert count==len(driver.calls)
        report['passed']=True
    finally:
        Path('/ws/output/v2_brain/mission_protocol_acceptance.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report));executor.shutdown();spin.join(timeout=2)
        for node in (coordinator,driver,probe):node.destroy_node()
        rclpy.shutdown();directory.cleanup()


if __name__=='__main__':main()
