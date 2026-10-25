"""Integration checks for pretrained-actor continuation and rollout observations."""
import unittest
import numpy as np
import torch
from stable_baselines3 import PPO
from qda_locomotion.transfer_env import B2TransferEnv
from qda_locomotion.transfer_ppo import initialize_actor
from qda_locomotion.upstream_b2 import UpstreamB2Policy


class TransferTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(2)
        cls.env=B2TransferEnv(randomize=False)
        cls.policy=UpstreamB2Policy('/ws/output/v2/upstream_b2',cls.env.plant)

    def test_original_features_match_actual_plant(self):
        obs,_=self.env.reset(seed=7,options={'command':[.15,.1,.25]})
        np.testing.assert_allclose(obs[:45],self.policy.observation(self.env.plant,self.env.command),atol=1e-6)
        for _ in range(5):
            action=self.policy.policy(torch.from_numpy(obs[:45])[None,:]).detach().numpy()[0]
            obs,_,_,_,_=self.env.step(action)
            self.policy.previous=action.copy()
            np.testing.assert_allclose(obs[:45],self.policy.observation(self.env.plant,self.env.command),atol=1e-6)

    def test_actor_warm_start_matches_reference(self):
        agent=PPO('MlpPolicy',self.env,device='cpu',n_steps=16,batch_size=16,
            policy_kwargs=dict(net_arch=dict(pi=[512,256,128],vf=[256,128]),activation_fn=torch.nn.ELU,ortho_init=False))
        self.assertLess(initialize_actor(agent,'/ws/output/v2/upstream_b2'),2e-5)

    def test_reset_reproducible_and_nonfinite_rejected(self):
        a,_=self.env.reset(seed=17,options={'command':[0,.1,0]})
        b,_=self.env.reset(seed=17,options={'command':[0,.1,0]})
        np.testing.assert_array_equal(a,b)
        self.assertEqual(len(a),78)
        with self.assertRaises(ValueError):self.env.step(np.full(12,np.nan))


if __name__=='__main__':unittest.main()
