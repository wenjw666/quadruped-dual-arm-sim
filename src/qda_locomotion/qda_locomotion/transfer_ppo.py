"""Warm-start PPO from the reviewed rl_sar B2 actor; save evidence before/after."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CheckpointCallback
from .transfer_env import B2TransferEnv
from .upstream_b2 import EXPECTED


def initialize_actor(agent, reference):
    path = Path(reference)/'policy.pt'
    if hashlib.sha256(path.read_bytes()).hexdigest() != EXPECTED['policy.pt']:
        raise ValueError('Unreviewed policy')
    upstream = torch.jit.load(str(path),map_location='cpu').eval()
    source = upstream.state_dict()
    target = agent.policy.state_dict()
    mapping = [('mlp_extractor.policy_net.0', 'actor.0'),
               ('mlp_extractor.policy_net.2', 'actor.2'),
               ('mlp_extractor.policy_net.4', 'actor.4'),('action_net', 'actor.6')]
    for dest, src in mapping:
        weight = source[src+'.weight']
        if dest.endswith('policy_net.0'):
            target[dest+'.weight'].zero_()
            target[dest+'.weight'][:,:45] = weight
        else:
            target[dest+'.weight'] = weight.clone()
        target[dest+'.bias'] = source[src+'.bias'].clone()
    agent.policy.load_state_dict(target)
    # Test the entire exporter, including normalizer; fails if the assumptions change.
    samples = torch.from_numpy(np.random.default_rng(20261025).normal(size=(128,78)).astype(np.float32))
    with torch.inference_mode():
        expected = upstream(samples[:,:45])
        actual = agent.policy.get_distribution(samples).distribution.mean
    error = float((expected-actual).abs().max())
    if error > 2e-5:
        raise RuntimeError(f'Warm-start actor parity failed: {error}')
    return error


def evaluate(agent, model, reference, seeds=(10007,10008,10009)):
    env = B2TransferEnv(model,reference=reference,randomize=True)
    commands = [('stand',[0,0,0]),('forward',[.15,0,0]),('lateral',[0,.1,0]),
                ('turn',[0,0,.25]),('forward_medium',[.4,0,0]),('backward',[-.15,0,0])]
    rows = []
    try:
        for seed in seeds:
            for name, command in commands:
                obs, reset_info = env.reset(seed=seed,options={'command':command})
                infos = []
                while True:
                    action,_ = agent.predict(obs,deterministic=True)
                    obs,_,terminated,truncated,info = env.step(action)
                    infos.append(info)
                    if terminated or truncated: break
                rows.append(dict(seed=seed,name=name,command=command,fallen=terminated,seconds=len(infos)*env.dt,
                    velocity_error_mps=float(np.mean([r['velocity_error_mps'] for r in infos])),
                    yaw_error_rps=float(np.mean([r['yaw_error_rps'] for r in infos])),
                    max_tilt_rad=max(r['tilt_rad'] for r in infos),
                    undesired_contact_frames=sum(r['undesired_contacts']>0 for r in infos),
                    joint_limit_clip_frames=sum(r['joint_limit_clip'] for r in infos),reset=reset_info))
    finally:
        env.close()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',default='/ws/output/v2/dynamics.xml')
    parser.add_argument('--reference',default='/ws/output/v2/upstream_b2')
    parser.add_argument('--output',required=True)
    parser.add_argument('--steps',type=int,default=65536)
    parser.add_argument('--envs',type=int,default=4)
    parser.add_argument('--seed',type=int,default=17)
    parser.add_argument('--resume')
    args = parser.parse_args()
    digest = hashlib.sha256(Path(args.model).read_bytes()).hexdigest()
    audit = json.loads(Path(args.model).with_suffix('.audit.json').read_text())
    if not audit.get('standing_gate_pass') or audit['model_sha256'] != digest:
        raise ValueError('Exact model must pass physics audit first')
    if args.steps < 1024 or args.envs < 1: raise ValueError('Invalid run size')
    out = Path(args.output); out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(2)
    factory = lambda: Monitor(B2TransferEnv(args.model,reference=args.reference))
    env = SubprocVecEnv([factory for _ in range(args.envs)],start_method='spawn') if args.envs>1 else DummyVecEnv([factory])
    probe = B2TransferEnv(args.model,reference=args.reference)
    contract = probe.contract; probe.close()
    (out/'contract.json').write_text(json.dumps(contract,indent=2))
    (out/'config.json').write_text(json.dumps(vars(args),indent=2))
    from . import transfer_env, upstream_b2, rl_env
    from qda_mujoco import dynamics
    sources = [Path(__file__), *[Path(m.__file__) for m in (transfer_env,upstream_b2,rl_env,dynamics)]]
    (out/'source_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
    parity = None
    if args.resume:
        if json.loads(Path(args.resume).parent.joinpath('contract.json').read_text()) != contract:
            raise ValueError('Resume contract mismatch')
        agent = PPO.load(args.resume,env=env,device='cpu')
    else:
        agent = PPO('MlpPolicy',env,seed=args.seed,device='cpu',n_steps=256,batch_size=256,n_epochs=5,
            learning_rate=3e-5,clip_range=.1,target_kl=.01,ent_coef=0.,
            policy_kwargs=dict(net_arch=dict(pi=[512,256,128],vf=[256,128]),activation_fn=torch.nn.ELU,
                               log_std_init=-2.,ortho_init=False),verbose=1)
        parity = initialize_actor(agent,args.reference)
    agent.set_logger(configure(str(out),['stdout','csv']))
    started = time.monotonic()
    try:
        before = evaluate(agent,args.model,args.reference)
        (out/'before.json').write_text(json.dumps(before,indent=2))
        agent.save(out/'initial_policy')
        agent.learn(total_timesteps=args.steps,reset_num_timesteps=not bool(args.resume),
                    callback=CheckpointCallback(save_freq=max(8192//args.envs,1),save_path=str(out/'checkpoints'),name_prefix='ppo'))
        agent.save(out/'policy')
        loaded = PPO.load(out/'policy',device='cpu')
        after = evaluate(loaded,args.model,args.reference)
        report = dict(algorithm='PPO warm-start from rl_sar B2 actor',initial_actor_max_abs_error=parity,
                      total_timesteps=agent.num_timesteps,wall_s=time.monotonic()-started,
                      before=before,after=after,runtime_ready=False,
                      limitations='Flat ground, arms held home; no sensor latency/noise or moving-arm validation')
        (out/'evaluation.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
    finally:
        env.close()


if __name__ == '__main__':
    main()
