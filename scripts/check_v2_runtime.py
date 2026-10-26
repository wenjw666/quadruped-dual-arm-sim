"""Read-only readiness check for the real graph after a complete launch restart."""
import json
import time
import urllib.request
from pathlib import Path
import rclpy
from qda_interfaces.srv import BrainRequest


def main():
    rclpy.init();node=rclpy.create_node('runtime_readiness_probe')
    client=node.create_client(BrainRequest,'/brain/request');end=time.monotonic()+180;result=None
    try:
        while time.monotonic()<end:
            if not client.wait_for_service(timeout_sec=2):continue
            f=client.call_async(BrainRequest.Request(request_json='{"op":"status"}'))
            rclpy.spin_until_future_complete(node,f,timeout_sec=8)
            if not f.done():continue
            result=json.loads(f.result().response_json);health=result.get('health',{})
            if (result.get('locomotion',{}).get('ready') and health.get('front_camera_fresh')
                and health.get('topics') and all(v.get('fresh') for v in health['topics'].values())):break
            time.sleep(.3)
        assert result and result.get('locomotion',{}).get('ready'),result
        assert not result.get('active') and not result.get('quarantine') and not result.get('mission_owner')
        assert not result.get('base_lease_owned') and not result['locomotion'].get('fault')
        health=result['health'];assert all(v['fresh'] for v in health['topics'].values()),health
        publishers=[p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
        assert publishers==['qda_command_mux'],publishers
        model=json.load(urllib.request.urlopen('http://127.0.0.1:8088/health',timeout=3));assert model['status']=='ok'
        report=dict(passed=True,scene_version=result['scene_version'],physics_time=result['locomotion'].get('physics_time'),
            fault=result['locomotion']['fault'],cmd_vel_publishers=publishers,model=model,
            node_names=sorted(node.get_node_names()),health=health,active=result['active'],
            robot_physics_signature=result['locomotion']['robot_physics_signature'])
        Path('/ws/output/v2_brain/runtime_acceptance.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='health'}),flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
