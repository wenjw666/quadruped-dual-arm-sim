"""Text/WAV to a proposal, or explicit idempotent skill submission to local ROS."""
import argparse
import json
import time
import rclpy
from qda_interfaces.srv import BrainRequest


def main():
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group()
    g.add_argument('--text');g.add_argument('--wav');g.add_argument('--status',action='store_true');g.add_argument('--pause',action='store_true')
    p.add_argument('--vlm',action='store_true');p.add_argument('--image',action='store_true')
    p.add_argument('--execute',action='store_true');p.add_argument('--task-id');a=p.parse_args()
    if a.execute and not a.task_id:p.error('--execute requires a stable --task-id for replay protection')
    rclpy.init();node=rclpy.create_node('brain_command');client=node.create_client(BrainRequest,'/brain/request')
    def call(payload):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('BRAIN_UNAVAILABLE')
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        rclpy.spin_until_future_complete(node,f,timeout_sec=60)
        if not f.done():raise TimeoutError('BRAIN_TIMEOUT')
        return json.loads(f.result().response_json)
    try:
        if a.status or a.pause:result=call({'op':'pause' if a.pause else 'status'})
        else:
            text=a.text;asr=None
            if a.wav:
                from qda_brain.voice import transcribe
                asr=transcribe(a.wav);text=asr['text']
                if asr['mean_word_confidence']<.75 or not text:raise ValueError('ASR_LOW_CONFIDENCE')
            if not text:p.error('provide --text, --wav, --status or --pause')
            result=call(dict(op='interpret',text=text,use_vlm=a.vlm,with_image=a.image))
            if asr:result['asr']=asr
            if a.execute and result.get('ok'):
                if not result.get('execution_allowed',False):raise ValueError('MODEL_PROPOSAL_NOT_VERIFIED_FOR_EXECUTION')
                proposal=result['proposal']
                if proposal['skill']=='pause':result['submission']=call({'op':'pause'})
                else:result['submission']=call(dict(op='submit',proposal=proposal,task_id=a.task_id,scene_version=result['scene_version']))
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result.get('ok') and result.get('submission',{'ok':True}).get('ok') else 2
    except Exception as exc:
        print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False));return 2
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
