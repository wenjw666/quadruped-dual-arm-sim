"""Submit and verify the physical task through the production brain API."""
import argparse,json,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from qda_interfaces.srv import BrainRequest

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ready-timeout',type=float,default=300.,help='Wall seconds to wait for model/ROS/backend startup');parser.add_argument('--out',default='/ws/output/grasp/ros-acceptance.json');parser.add_argument('--timeout',type=float,default=2400.);args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('physical_pick_place_operator')
    client=node.create_client(BrainRequest,'/brain/request')
    report=dict(success=False,source='production brain API and live physical telemetry',samples=[])
    last=[None,0.]
    def state(msg):
        s=json.loads(msg.data)
        if s['stage']!=last[0] or time.monotonic()-last[1]>2:
            report['samples'].append(s)
            if s['stage']!=last[0]:print(json.dumps(s),flush=True)
            last[:]=[s['stage'],time.monotonic()]
    node.create_subscription(String,'/manipulation/status',state,10)
    def call(payload,timeout=15):
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('BRAIN_UNAVAILABLE')
        f=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)))
        rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
        if not f.done():raise RuntimeError('BRAIN_REQUEST_TIMEOUT')
        return json.loads(f.result().response_json)
    tid='physical-pick-place-'+str(time.time_ns());submitted=False
    try:
        until=time.monotonic()+args.ready_timeout
        while time.monotonic()<until:
            if not client.wait_for_service(timeout_sec=10):continue
            snapshot=call({'op':'status'})
            if (snapshot.get('capabilities',{}).get('physical_pick_place')
                and snapshot.get('locomotion',{}).get('ready')
                and snapshot['locomotion'].get('physics_time',0.) >= 30.):break
            rclpy.spin_once(node,timeout_sec=.5)
        else:raise RuntimeError('PHYSICAL_TASK_NOT_READY')
        reply=call(dict(op='submit',task_id=tid,scene_version=snapshot['scene_version'],
                        proposal=dict(skill='pick_place',object_id='red_cube',target_id='place_station',arm='left')))
        if not reply.get('accepted'):raise RuntimeError(str(reply))
        submitted=True;until=time.monotonic()+args.timeout
        while time.monotonic()<until:
            rows=call({'op':'status','task_id':tid}).get('tasks',[])
            if rows and rows[0]['status'] not in ('RESERVED','RUNNING'):
                report['task']=rows[0];report['success']=rows[0]['status']=='SUCCEEDED'
                # Let the post-release telemetry arrive as well: a task journal
                # success must not hide a stale-cleanup error in the backend.
                until_state=time.monotonic()+2
                while time.monotonic()<until_state:rclpy.spin_once(node,timeout_sec=.05)
                if report['success']:
                    final=report['samples'][-1] if report['samples'] else {}
                    report['success']=final.get('stage')=='placed' and final.get('completed') and not final.get('error')
                    if not report['success']:report['error']='POST_TASK_STATE_INVALID'
                break
            for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
        else:raise RuntimeError('OPERATOR_TIMEOUT')
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc) or 'CANCELLED_BY_OPERATOR'
        if submitted:
            try:call({'op':'pause'})
            except Exception:pass
    finally:
        report['task_id']=tid
        out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if report['success'] else 1
if __name__=='__main__':raise SystemExit(main())
