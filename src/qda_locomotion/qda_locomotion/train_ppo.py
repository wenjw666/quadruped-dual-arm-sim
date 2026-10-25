"""Reproducible PPO smoke/standing/velocity runs; checkpoints never imply acceptance."""
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv,SubprocVecEnv
from stable_baselines3.common.logger import configure
from .rl_env import B2Env


def main():
    p=argparse.ArgumentParser();p.add_argument('--model',default='/ws/output/v2/dynamics.xml')
    p.add_argument('--task',choices=['stand','velocity'],default='stand');p.add_argument('--steps',type=int,default=8192)
    p.add_argument('--envs',type=int,default=2);p.add_argument('--seed',type=int,default=7)
    p.add_argument('--output',required=True);p.add_argument('--resume');p.add_argument('--eval-episodes',type=int,default=5)
    a=p.parse_args()
    if a.steps<512 or a.envs<1 or a.eval_episodes<1:raise ValueError('Invalid run size')
    audit=json.loads(Path(a.model).with_suffix('.audit.json').read_text())
    digest=hashlib.sha256(Path(a.model).read_bytes()).hexdigest()
    if not audit.get('standing_gate_pass') or audit.get('model_sha256')!=digest:
        raise RuntimeError('Run and pass audit_v2_dynamics.py on this exact model before PPO')
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(2)
    def factory():return Monitor(B2Env(a.model,a.task,randomize=True))
    env=SubprocVecEnv([factory for _ in range(a.envs)],start_method='spawn') if a.envs>1 else DummyVecEnv([factory])
    probe=B2Env(a.model,a.task);contract=probe.contract
    (out/'contract.json').write_text(json.dumps(contract,indent=2))
    (out/'config.json').write_text(json.dumps(vars(a),indent=2))
    source_files=[Path(__file__),Path(__file__).with_name('rl_env.py')]
    from qda_mujoco import dynamics
    source_files.append(Path(dynamics.__file__))
    (out/'source_sha256.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files},indent=2))
    if a.resume:
        previous=json.loads(Path(a.resume).parent.joinpath('contract.json').read_text())
        if previous!=contract:raise ValueError('Resume contract/model mismatch')
        agent=PPO.load(a.resume,env=env,device='cpu')
    else:
        agent=PPO('MlpPolicy',env,device='cpu',seed=a.seed,n_steps=256,batch_size=128,n_epochs=5,
                  learning_rate=3e-4,clip_range=.2,target_kl=.02,ent_coef=.005,
                  policy_kwargs=dict(net_arch=dict(pi=[128,128],vf=[128,128]),log_std_init=-1.5),verbose=1)
    initial={k:v.detach().clone() for k,v in agent.policy.state_dict().items()}
    agent.set_logger(configure(str(out),['stdout','csv']))
    started=time.monotonic()
    try:
        agent.learn(total_timesteps=a.steps,reset_num_timesteps=not bool(a.resume))
        agent.save(out/'policy')
        changed=any(not torch.equal(initial[k],v) for k,v in agent.policy.state_dict().items())
        assert changed,'Optimizer did not change parameters'
        loaded=PPO.load(out/'policy',device='cpu')
        rows=[]
        for i in range(a.eval_episodes):
            obs,_=probe.reset(seed=10000+a.seed+i);infos=[];total=0.
            while True:
                act,_=loaded.predict(obs,deterministic=True)
                obs,reward,terminated,truncated,info=probe.step(act);infos.append(info);total+=reward
                if terminated or truncated:break
            rows.append(dict(seed=10000+a.seed+i,return_=total,steps=len(infos),fallen=terminated,
                             mean_velocity_error=float(np.mean([x['velocity_error_mps'] for x in infos])),
                             max_tilt_rad=max(x['tilt_rad'] for x in infos),min_height=min(x['height'] for x in infos)))
        report=dict(task=a.task,algorithm='Stable-Baselines3 PPO',optimizer_updated=changed,checkpoint_reload=True,
                    total_timesteps=agent.num_timesteps,wall_s=time.monotonic()-started,evaluation=rows,
                    runtime_ready=False,note='Pipeline validation only; not 30-round task acceptance or X5 deployment')
        (out/'evaluation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    finally:env.close();probe.close()


if __name__=='__main__':main()
