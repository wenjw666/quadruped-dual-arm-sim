"""Run the sequence through real Nav2, skills, MoveIt and MuJoCo (domain 8)."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from qda_interfaces.srv import BrainRequest
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from make_mission_plan import step
from qda_brain.contracts import SCENE_VERSION


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preset',choices=['handoff','dual_visual'],default='handoff')
    parser.add_argument('--output',help='Unique report path; omit for the historical acceptance location')
    args=parser.parse_args();rclpy.init();node=rclpy.create_node('coordination_acceptance')
    client=node.create_client(BrainRequest,'/brain/request');owners=[];metrics=[];state={};state_times=[]
    def owner(msg):
        data=json.loads(msg.data)
        if not owners or any(data.get(k)!=owners[-1].get(k) for k in ('owner','lease_owner','lease_mode','reason')):owners.append(data)
    node.create_subscription(String,'/motion/command_owner',owner,10)
    node.create_subscription(String,'/brain/servo_metrics',lambda m:metrics.append(json.loads(m.data)),10)
    def on_state(msg):state.update(json.loads(msg.data));state_times.append(time.monotonic())
    node.create_subscription(String,'/locomotion/rl_status',on_state,10)
    manual=node.create_publisher(Twist,'/cmd_vel_manual',10)
    def call(payload):
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('BRAIN_UNAVAILABLE')
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        rclpy.spin_until_future_complete(node,f,timeout_sec=10)
        if not f.done():raise TimeoutError('BRAIN_REQUEST')
        return json.loads(f.result().response_json)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    report={'preset':args.preset,'passed':False,'localization':'simulation_truth','physical_pick_place':False}
    mid='coord-'+args.preset+'-'+str(int(time.time()));report['mission_id']=mid
    try:
        end=time.monotonic()+150
        while time.monotonic()<end:
            spin(.3)
            if state.get('ready') and not state.get('fault'):break
        assert state.get('ready') and not state.get('fault'),state
        skills=[step('navigate',target='start')]
        skills+=([step('home','left'),step('home','right')] if args.preset=='handoff' else [step('visual_align','left'),step('visual_align','right')])
        skills+=[step('inspect')];plan=dict(scene_version=SCENE_VERSION,steps=skills)
        bad={**plan,'steps':[step('inspect'),dict(skill='pick_place',object_id='red_cube',target_id='place_station',arm='left')]}
        assert not call(dict(op='mission_submit',mission_id=mid+'-bad',plan=bad))['ok']
        report['preflight_rejects_unavailable_step']=True
        assert call(dict(op='mission_submit',mission_id=mid,plan=plan))['accepted']
        interference_checked=False;end=time.monotonic()+1000
        while time.monotonic()<end:
            spin(.25);result=call(dict(op='mission_status',mission_id=mid));rows=result['missions']
            if rows and rows[0]['status'] not in ('RESERVED','RUNNING'):
                report['mission']=rows[0];break
            status=call({'op':'status'})
            if status.get('mission_owner')==mid and not interference_checked:
                external=call(dict(op='submit',task_id=mid+'-outside',scene_version=SCENE_VERSION,proposal=step('inspect')))
                assert not external.get('accepted',False),'External skill entered mission'
                report['external_skill_rejected']=True;interference_checked=True
            if state.get('fault'):raise AssertionError(state['fault'])
        assert report.get('mission',{}).get('status')=='SUCCEEDED',report.get('mission')
        assert report.get('external_skill_rejected'),report
        spin(.5);final=call({'op':'status'});report['final_state']=final['locomotion'];report['owner_transitions']=owners
        report['servo_samples']=metrics;report['health']=final['health']
        report['max_status_receive_gap_wall_s']=max((b-a for a,b in zip(state_times,state_times[1:])),default=0.)
        assert not final['active'] and not final['quarantine'] and not final['base_lease_owned'] and not final['mission_owner']
        assert any(o.get('lease_mode')=='nav' for o in owners)
        assert any(o.get('lease_mode')=='hold' for o in owners)
        # Replay is read-only: step IDs/results remain byte-for-byte equal.
        assert call(dict(op='mission_submit',mission_id=mid,plan=plan))['duplicate']
        replay=call(dict(op='mission_status',mission_id=mid))['missions'][0]
        assert replay==report['mission'];report['terminal_replay_unchanged']=True
        report['passed']=True
    finally:
        report['servo_samples']=metrics;report['owner_transitions']=owners;report['last_state']=state
        if not report['passed']:
            try:call({'op':'pause'})
            except Exception:pass
        target=Path(args.output or '/ws/output/v2_brain/coordination_'+args.preset+'_acceptance.json')
        target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('owner_transitions','servo_samples','health','final_state')},ensure_ascii=False),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
