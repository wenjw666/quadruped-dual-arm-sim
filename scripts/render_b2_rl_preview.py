"""Render actual free-base simulation: stand, forward, lateral, turn, stop."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import json
from pathlib import Path
import cv2
import mujoco
import numpy as np
import torch
from qda_locomotion.transfer_env import B2TransferEnv
from qda_locomotion.upstream_b2 import UpstreamB2Policy
from qda_locomotion.velocity_adapter import VelocityAdapter


def render(output,mode):
    torch.set_num_threads(2)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise FileExistsError(output)
    env=B2TransferEnv(randomize=False,episode_seconds=16.)
    obs,_=env.reset(seed=41007,options={'command':[0,0,0]})
    actor=UpstreamB2Policy('/ws/output/v2/upstream_b2',env.plant)
    adapter=VelocityAdapter(env.plant,'/ws/output/v2_audit/b2_command_response.json',env.contract['model_sha256'],feedback=mode=='contact_feedback')
    renderer=mujoco.Renderer(env.plant.model,height=480,width=640)
    camera=mujoco.MjvCamera();camera.distance=3.4;camera.azimuth=135;camera.elevation=-22
    options=mujoco.MjvOption();options.geomgroup[1]=0;options.geomgroup[2]=1
    writer=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*'mp4v'),25.,(640,480))
    if not writer.isOpened():raise RuntimeError('Video writer unavailable')
    rows=[]
    try:
        for step in range(800):
            seconds=step*.02
            command=([0,0,0] if seconds<2 or seconds>=13 else [.15,0,0] if seconds<6
                     else [0,.1,0] if seconds<10 else [0,0,.25])
            env.command=np.asarray(command,dtype=float)
            obs=env.observation()
            policy_command,_=adapter.command(command,env.plant)
            if mode!='original':obs[6:9]=policy_command
            with torch.inference_mode():action=actor.policy(torch.from_numpy(obs[:45])[None,:]).numpy()[0]
            obs,_,fallen,_,info=env.step(action)
            rows.append(dict(seconds=seconds,command=command,policy_command=policy_command.tolist(),**info))
            if step%2==0:
                camera.lookat[:]=env.plant.data.xpos[env.plant.base_id]+np.array([0,0,.35])
                renderer.update_scene(env.plant.data,camera=camera,scene_option=options)
                frame=renderer.render()
                writer.write(cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
                if step==300:cv2.imwrite(str(output.with_suffix('.png')),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
            if fallen:break
    finally:
        writer.release();renderer.close();env.close()
    report=dict(mode=mode,steps=len(rows),fallen=rows[-1]['fallen'],runtime_ready=False,
        timeline='0-2s stand; 2-6s forward .15m/s; 6-10s left .1m/s; 10-13s turn .25rad/s; 13-16s stop',rows=rows)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['original','feedforward','contact_feedback'],default='contact_feedback')
    p.add_argument('--output',required=True)
    a=p.parse_args();render(a.output,a.mode)
