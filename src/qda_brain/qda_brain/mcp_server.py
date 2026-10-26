"""Offline-testable MCP stdio adapter; no cloud credentials or arbitrary execution."""
import json
import time
import threading
import sys
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
from mcp.server.fastmcp import FastMCP
from qda_interfaces.srv import BrainRequest


def main():
    rclpy.init();node=rclpy.create_node('robot_mcp_bridge')
    client=node.create_client(BrainRequest,'/brain/request');lock=threading.Lock()
    executor=SingleThreadedExecutor();executor.add_node(node)
    def spin():
        try:executor.spin()
        except ExternalShutdownException:pass
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    print('Robot MCP ROS_DOMAIN_ID='+os.environ.get('ROS_DOMAIN_ID','0'),file=sys.stderr,flush=True)
    mcp=FastMCP('qda-local-robot')
    def call(payload):
        with lock:
            if not client.wait_for_service(timeout_sec=2):return {'ok':False,'error':'BRAIN_UNAVAILABLE'}
            f=client.call_async(BrainRequest.Request(request_json=json.dumps(payload)));end=time.monotonic()+8
            while not f.done() and time.monotonic()<end:time.sleep(.02)
            if not f.done():client.remove_pending_request(f);return {'ok':False,'error':'TIMEOUT'}
            return json.loads(f.result().response_json)
    @mcp.tool()
    def status(task_id:str='')->dict:
        """Read current task and robot state."""
        return call({'op':'status',**({'task_id':task_id} if task_id else {})})
    @mcp.tool()
    def interpret_text(text:str)->dict:
        """Interpret a supported local phrase without executing movement."""
        return call({'op':'interpret','text':text})
    @mcp.tool()
    def submit_skill(task_id:str,scene_version:int,skill:str,object_id:str='none',target_id:str='none',arm:str='none')->dict:
        """Submit one whitelisted skill with an idempotency key and current scene version."""
        return call({'op':'submit','task_id':task_id,'scene_version':scene_version,
                     'proposal':dict(skill=skill,object_id=object_id,target_id=target_id,arm=arm)})
    @mcp.tool()
    def pause()->dict:
        """Cancel the active skill and hold the base; never open grippers."""
        return call({'op':'pause'})
    @mcp.tool()
    def cancel()->dict:
        """Cancel the active skill and hold position."""
        return call({'op':'cancel'})
    @mcp.tool()
    def submit_mission(mission_id:str,scene_version:int,steps:list[dict])->dict:
        """Submit 1..12 validated skills in order; stop on the first failed step."""
        return call({'op':'mission_submit','mission_id':mission_id,'plan':{'scene_version':scene_version,'steps':steps}})
    @mcp.tool()
    def mission_status(mission_id:str='')->dict:
        """Read sequence progress and each step's recorded result."""
        return call({'op':'mission_status',**({'mission_id':mission_id} if mission_id else {})})
    @mcp.tool()
    def cancel_mission(mission_id:str)->dict:
        """Cancel a sequence and wait asynchronously for its child to stop."""
        return call({'op':'mission_cancel','mission_id':mission_id})
    try:mcp.run(transport='stdio')
    finally:
        executor.shutdown();thread.join(timeout=3);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
