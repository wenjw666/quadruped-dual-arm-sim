"""Measure upstream-policy response before choosing a low-speed adaptation method."""
import argparse
import json
import hashlib
from pathlib import Path
import numpy as np
import torch
from qda_locomotion.transfer_env import B2TransferEnv
from qda_locomotion.upstream_b2 import UpstreamB2Policy,EXPECTED


def probe(model, output):
    torch.set_num_threads(2)
    env=B2TransferEnv(model,randomize=False,episode_seconds=6.)
    actor=UpstreamB2Policy('/ws/output/v2/upstream_b2',env.plant)
    rows=[]
    for axis,values in [(0,[-.6,-.4,-.25,.15,.25,.35,.45,.6]),
                        (1,[-.6,-.4,-.25,.1,.25,.35,.45,.6]),
                        (2,[-.75,-.5,-.25,.25,.5,.75])]:
        command=np.zeros(3);command[axis]=values[0]
        for value in values:
            command[axis]=value
            obs,_=env.reset(seed=41,options={'command':command})
            actor.previous[:]=0.
            speeds=[];tilts=[]
            for step in range(300):
                with torch.inference_mode():
                    action=actor.policy(torch.from_numpy(obs[:45])[None,:]).numpy()[0]
                obs,_,fallen,_,info=env.step(action)
                if step>=100:speeds.append([*info['body_velocity'],info['body_yaw_rate']])
                tilts.append(info['tilt_rad'])
                if fallen:break
            row=dict(axis=axis,command=float(value),fallen=fallen,seconds=(step+1)*.02,
                     mean_actual=np.mean(speeds,axis=0).tolist() if speeds else None,
                     max_tilt_rad=max(tilts))
            rows.append(row);print(json.dumps(row),flush=True)
    env.close()
    Path(output).write_text(json.dumps(dict(model=model,model_sha256=hashlib.sha256(Path(model).read_bytes()).hexdigest(),
        policy_sha256=EXPECTED['policy.pt'],seed=41,rows=rows,
        note='Diagnostic response sweep; commands are not a validated operating envelope'),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',default='/ws/output/v2/dynamics.xml')
    p.add_argument('--output',default='/ws/output/v2_audit/b2_command_response.json')
    a=p.parse_args();probe(a.model,a.output)
