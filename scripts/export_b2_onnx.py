"""Export a transfer PPO actor and verify ONNX Runtime on actual rollout observations."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import onnx
import onnxruntime as ort
import torch
from stable_baselines3 import PPO
from qda_locomotion.transfer_env import B2TransferEnv


class DeterministicActor(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.features = policy.mlp_extractor.policy_net
        self.action = policy.action_net

    def forward(self, observation):
        return self.action(self.features(observation))


def export(checkpoint, model, reference, destination):
    torch.set_num_threads(2)
    checkpoint = Path(checkpoint)
    contract = json.loads(checkpoint.parent.joinpath('contract.json').read_text())
    env = B2TransferEnv(model,reference=reference,randomize=True)
    if contract != env.contract:
        raise ValueError('Checkpoint contract does not match environment/model')
    agent = PPO.load(checkpoint,device='cpu')
    actor = DeterministicActor(agent.policy).eval()
    reference_actor=torch.jit.load(str(Path(reference)/'policy.pt'),map_location='cpu').eval()
    parity_samples=torch.from_numpy(np.random.default_rng(101).normal(size=(128,78)).astype(np.float32))
    with torch.inference_mode():
        reference_error=float((actor(parity_samples)-reference_actor(parity_samples[:,:45])).abs().max())
    observations = []
    try:
        for seed, command in [(20007,[0,0,0]),(20008,[.15,0,0]),(20009,[0,.1,0]),(20010,[0,0,.25])]:
            obs,_ = env.reset(seed=seed,options={'command':command})
            for _ in range(250):
                observations.append(obs.copy())
                action,_ = agent.predict(obs,deterministic=True)
                obs,_,terminated,truncated,_ = env.step(action)
                if terminated or truncated: break
    finally:
        env.close()
    destination = Path(destination); destination.parent.mkdir(parents=True,exist_ok=True)
    if destination.exists(): raise FileExistsError(destination)
    torch.onnx.export(actor,torch.zeros(1,78),str(destination),opset_version=17,dynamo=False,
                      input_names=['observation'],output_names=['action'],
                      dynamic_axes={'observation':{0:'batch'},'action':{0:'batch'}})
    onnx.checker.check_model(onnx.load(str(destination)))
    options = ort.SessionOptions(); options.intra_op_num_threads = 1; options.inter_op_num_threads = 1
    runtime = ort.InferenceSession(str(destination),sess_options=options,providers=['CPUExecutionProvider'])
    samples = np.asarray(observations,dtype=np.float32)
    with torch.inference_mode():
        expected = actor(torch.from_numpy(samples)).numpy()
    actual = runtime.run(['action'],{'observation':samples})[0]
    max_error = float(np.max(np.abs(expected-actual)))
    np.testing.assert_allclose(actual,expected,atol=2e-5,rtol=2e-5)
    elapsed = []
    for i in range(1100):
        start = time.perf_counter_ns()
        runtime.run(['action'],{'observation':samples[i%len(samples):i%len(samples)+1]})
        if i>=100: elapsed.append((time.perf_counter_ns()-start)*1e-6)
    report = dict(onnx_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                  reference_equivalent=reference_error<2e-5,reference_actor_max_abs_error=reference_error,
                  checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                  contract=contract,opset=17,rollout_samples=len(samples),max_abs_error=max_error,
                  desktop_cpu_inference_ms_p50=float(np.percentile(elapsed,50)),
                  desktop_cpu_inference_ms_p99=float(np.percentile(elapsed,99)),
                  runtime_ready=False,hardware='Desktop Docker CPU; not RDK X5 measurements')
    destination.with_suffix('.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='contract'},indent=2))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint',required=True)
    p.add_argument('--model',default='/ws/output/v2/dynamics.xml')
    p.add_argument('--reference',default='/ws/output/v2/upstream_b2')
    p.add_argument('--output',required=True)
    a=p.parse_args();export(a.checkpoint,a.model,a.reference,a.output)
