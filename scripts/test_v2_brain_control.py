"""Live task cancellation, replay and physical-grasp capability gate."""
import json
import time
from pathlib import Path
import rclpy
from qda_interfaces.srv import BrainRequest


def main():
    rclpy.init();node=rclpy.create_node('brain_control_acceptance');client=node.create_client(BrainRequest,'/brain/request')
    def call(payload):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('NO_BRAIN')
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        rclpy.spin_until_future_complete(node,f,timeout_sec=10)
        if not f.done():raise TimeoutError('REQUEST_TIMEOUT')
        return json.loads(f.result().response_json)
    def terminal(task_id):
        end=time.monotonic()+20
        while time.monotonic()<end:
            rows=call({'op':'status','task_id':task_id})['tasks']
            if rows and rows[0]['status'] not in ('RESERVED','RUNNING'):return rows[0]
            time.sleep(.1)
        raise TimeoutError('CANCEL_DID_NOT_FINISH')
    report={};prefix='control-'+str(int(time.time()))
    try:
        state=call({'op':'status'});version=state['scene_version'];assert state['active'] is None
        goal=dict(skill='navigate',object_id='none',target_id='north_entry',arm='none')
        submission=dict(op='submit',task_id=prefix+'-nav',scene_version=version,proposal=goal)
        assert call(submission)['accepted'];assert call({'op':'pause'})['ok']
        report['cancelled']=terminal(submission['task_id']);assert report['cancelled']['status']=='CANCELLED'
        assert call(submission)['accepted'];report['cancel_replay']=terminal(submission['task_id'])
        assert report['cancel_replay']==report['cancelled']
        gate=dict(skill='pick_place',object_id='red_cube',target_id='place_station',arm='right')
        assert call(dict(op='submit',task_id=prefix+'-pick',scene_version=version,proposal=gate))['accepted']
        report['grasp_gate']=terminal(prefix+'-pick')
        assert report['grasp_gate']['result']['error_code']=='RIGHT_ARM_PHYSICAL_TASK_NOT_VALIDATED'
        report['voice_task']=call({'op':'status','task_id':'offline-voice-status-001'})['tasks']
        assert report['voice_task'][0]['status']=='SUCCEEDED'
        report['passed']=True
    finally:
        Path('/ws/output/v2_brain/control_acceptance.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
