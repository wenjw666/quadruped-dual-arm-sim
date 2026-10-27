"""Visible, recorded use of the existing brain/mission API; no invented planner output.

Each explicit input sentence is interpreted independently. The caller assembles
the ordered proposals and uses the production plan validator before submitting.
Model disagreement aborts; there is no silent rules fallback.
"""
import argparse
import json
import time
import signal
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from qda_interfaces.srv import BrainRequest
from qda_brain.contracts import SCENE_VERSION
from qda_brain.mission import validate_plan


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['dual_visual','inspection','brain_home'],required=True)
    parser.add_argument('--out',required=True)
    parser.add_argument('--vlm',action='store_true')
    parser.add_argument('--status-wav',help='Offline WAV used for the status step, never microphone capture')
    parser.add_argument('--record-to',help='Start/finish an owned read-only recorder in a new directory')
    args=parser.parse_args()
    target=Path(args.out)
    if target.exists():raise FileExistsError(target)
    target.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('live_demo_operator')
    pub=node.create_publisher(String,'/demo/brain_trace',QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client=node.create_client(BrainRequest,'/brain/request')
    report={'mode':args.mode,'passed':False,'source':'actual brain and mission service responses','events':[]}
    mid='live-'+args.mode+'-'+str(int(time.time()));submitted=False;recorder=None;record_log=None
    def event(phase,**data):
        row=dict(phase=phase,unix=time.time(),mission_id=mid,**data)
        report['events'].append(row);pub.publish(String(data=json.dumps(row,ensure_ascii=False)))
        print(json.dumps(row,ensure_ascii=False),flush=True)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def call(payload,timeout=65):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('BRAIN_UNAVAILABLE')
        future=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('BRAIN_TIMEOUT')
        return json.loads(future.result().response_json)
    phrases={'dual_visual':['前往起点','左臂对准','右臂对准','查询状态'],
             'inspection':['前往检查点','查询状态','前往起点'],
             'brain_home':['左臂回位','让右边的机械臂回到初始位置','查询状态']}[args.mode]
    try:
        if args.record_to:
            record_root=Path(args.record_to)
            if record_root.exists():raise FileExistsError(record_root)
            record_root.parent.mkdir(parents=True,exist_ok=True)
            record_log=record_root.with_suffix('.log').open('w')
            recorder=subprocess.Popen([sys.executable,str(Path(__file__).with_name('record_product_demo.py')),
                '--out',str(record_root),'--seconds','1800'],stdout=record_log,stderr=subprocess.STDOUT)
        spin(2)
        deadline=time.monotonic()+180
        while time.monotonic()<deadline:
            status=call({'op':'status'})
            health=status.get('health',{})
            if (status.get('locomotion',{}).get('ready') and not status['locomotion'].get('fault')
                and not status.get('active') and not status.get('quarantine') and not status.get('mission_owner')
                and health.get('front_camera_fresh') and health.get('topics')
                and all(row.get('fresh') for row in health['topics'].values())):break
            spin(1)
        else:raise RuntimeError('RUNTIME_NOT_READY')
        if recorder:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            camera=node.create_client(SetParameters,'/qda_mujoco/set_parameters')
            if not camera.wait_for_service(timeout_sec=5):raise RuntimeError('CAMERA_NODE_UNAVAILABLE')
            f=camera.call_async(SetParameters.Request(parameters=[Parameter(name='demo_camera',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=True))]))
            rclpy.spin_until_future_complete(node,f,timeout_sec=10)
            if not f.done() or not all(r.successful for r in f.result().results):raise RuntimeError('CAMERA_ENABLE_FAILED')
            deadline=time.monotonic()+60
            while time.monotonic()<deadline:
                if recorder.poll() is not None:raise RuntimeError('RECORDER_EXITED')
                if len(list((record_root/'follow').glob('*.jpg')))>=3:break
                spin(.3)
            else:raise RuntimeError('NO_LIVE_CAMERA_FRAMES')
        if args.status_wav:
            from qda_brain.voice import transcribe
            from qda_brain.contracts import parse_local
            event('transcribing',file=args.status_wav,source='offline WAV input')
            asr=transcribe(args.status_wav)
            event('recognized',response=asr)
            if asr.get('mean_word_confidence',0)<.75 or parse_local(asr['text'])['skill']!='inspect':
                raise RuntimeError('ASR_STATUS_NOT_VERIFIED')
            phrases[phrases.index('查询状态')]=asr['text']
        event('input',sentences=phrases,plan_source='operator_ordered_sentences',use_vlm=args.vlm)
        proposals=[]
        for index,text in enumerate(phrases):
            event('interpreting',index=index,text=text)
            # When VLM mode is selected, send the current live camera frame as
            # the grounded observation. The trace records both the image flag
            # and the constrained proposal returned by the model.
            result=call(dict(op='interpret',text=text,use_vlm=args.vlm,with_image=args.vlm))
            event('proposal',index=index,text=text,response=result)
            if not result.get('ok') or not result.get('execution_allowed'):
                raise RuntimeError('PROPOSAL_REJECTED: '+json.dumps(result,ensure_ascii=False))
            proposals.append(result['proposal']);spin(1)
        plan=validate_plan(dict(scene_version=SCENE_VERSION,steps=proposals))
        event('validated_plan',plan=plan,plan_source='ordered_validated_proposals')
        spin(2)
        result=call(dict(op='mission_submit',mission_id=mid,plan=plan))
        event('submitted',response=result)
        if not result.get('accepted'):raise RuntimeError('MISSION_REJECTED')
        submitted=True;end=time.monotonic()+1400;previous=None
        while time.monotonic()<end:
            spin(.5)
            state=call(dict(op='mission_status',mission_id=mid))
            rows=state.get('missions',[])
            if not rows:continue
            row=rows[0];key=json.dumps(row,sort_keys=True)
            if key!=previous:event('execution',mission=row);previous=key
            if row['status'] not in ('RESERVED','RUNNING'):
                report['mission']=row;report['passed']=row['status']=='SUCCEEDED';break
        else:raise TimeoutError('MISSION_TIMEOUT')
        event('finished',passed=report['passed']);spin(3)
    except Exception as exc:
        event('failed',error=str(exc))
        if submitted:
            try:event('cancel_requested',response=call(dict(op='mission_cancel',mission_id=mid)))
            except Exception as cancel:report['cancel_error']=str(cancel)
    finally:
        if recorder and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=30)
            except subprocess.TimeoutExpired:
                report['recorder_error']='RECORDER_STOP_TIMEOUT'
                recorder.terminate();recorder.wait(timeout=10)
        if record_log:record_log.close()
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
