"""Compare original commands, calibrated feed-forward, and contact-feedback adaptation."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from qda_locomotion.transfer_env import B2TransferEnv
from qda_locomotion.upstream_b2 import UpstreamB2Policy
from qda_locomotion.velocity_adapter import VelocityAdapter


def evaluate(model,response,output,modes=('original','feedforward','contact_feedback'),seeds=(31007,31008,31009),moving_arms=False):
    torch.set_num_threads(2)
    env=B2TransferEnv(model,randomize=True,episode_seconds=10.)
    actor=UpstreamB2Policy('/ws/output/v2/upstream_b2',env.plant)
    nominal_home=env.plant.home.copy()
    rows=[]
    commands=[('stand',[0,0,0]),('forward',[.15,0,0]),('backward',[-.15,0,0]),
              ('left',[0,.1,0]),('right',[0,-.1,0]),('turn_left',[0,0,.25]),('turn_right',[0,0,-.25])]
    for mode in modes:
        for seed in seeds:
            for name,command in commands:
                env.plant.home[:]=nominal_home
                obs,_=env.reset(seed=seed,options={'command':command})
                adapter=VelocityAdapter(env.plant,response,env.contract['model_sha256'],feedback=mode=='contact_feedback')
                infos=[];estimate_errors=[];valid=0
                while True:
                    if moving_arms:
                        # Real moving arm targets, not inertial edits or pose overwrites.
                        t=env.steps*.02
                        for side,phase in [('left',0.),('right',np.pi/2)]:
                            for joint,amplitude in [(1,.08),(4,.04)]:
                                index=env.plant.names.index(f'{side}_joint_link{joint}')
                                env.plant.home[index]=nominal_home[index]+amplitude*np.sin(2*np.pi*.25*t+phase)
                    actual_command,estimate=adapter.command(command,env.plant)
                    if mode!='original':obs[6:9]=actual_command
                    if estimate['contact_valid']:
                        truth=env.plant.state()['velocity'][3:5]
                        estimate_errors.append(float(np.linalg.norm(np.array(estimate['estimated_velocity'][:2])-truth)))
                        valid+=1
                    with torch.inference_mode():action=actor.policy(torch.from_numpy(obs[:45])[None,:]).numpy()[0]
                    obs,_,fallen,truncated,info=env.step(action);infos.append(info)
                    if fallen or truncated:break
                row=dict(mode=mode,seed=seed,name=name,fallen=fallen,seconds=len(infos)*.02,
                    velocity_error_mps=float(np.mean([r['velocity_error_mps'] for r in infos])),
                    yaw_error_rps=float(np.mean([r['yaw_error_rps'] for r in infos])),
                    max_tilt_rad=max(r['tilt_rad'] for r in infos),
                    contact_estimator_valid_fraction=valid/len(infos),
                    estimator_error_mps=float(np.mean(estimate_errors)) if estimate_errors else None,
                    undesired_contact_frames=sum(r['undesired_contacts']>0 for r in infos))
                row['last_2s_velocity_error_mps']=float(np.mean([r['velocity_error_mps'] for r in infos[-100:]]))
                row['last_2s_yaw_error_rps']=float(np.mean([r['yaw_error_rps'] for r in infos[-100:]]))
                rows.append(row);print(json.dumps(row),flush=True)
    env.close()
    Path(output).write_text(json.dumps(dict(model_sha256=hashlib.sha256(Path(model).read_bytes()).hexdigest(),
        response_sha256=hashlib.sha256(Path(response).read_bytes()).hexdigest(),rows=rows,runtime_ready=False,
        moving_arms=moving_arms,arm_motion='joint1 +/-0.08rad, joint4 +/-0.04rad, 0.25Hz, phases 0/pi2' if moving_arms else 'home',
        note='Same pretrained actor; command calibration and optional contact-kinematic PI feedback, not a new trained PPO policy'),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',default='/ws/output/v2/dynamics.xml')
    p.add_argument('--response',default='/ws/output/v2_audit/b2_command_response.json')
    p.add_argument('--output',default='/ws/output/v2_audit/b2_velocity_adapter.json')
    p.add_argument('--final-validation',action='store_true',help='5 fresh seeds, contact feedback only, moving arms')
    a=p.parse_args()
    if a.final_validation:evaluate(a.model,a.response,a.output,modes=('contact_feedback',),seeds=range(42007,42012),moving_arms=True)
    else:evaluate(a.model,a.response,a.output)
