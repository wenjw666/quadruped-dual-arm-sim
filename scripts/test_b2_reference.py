"""Check named motor mapping and observation/action conventions against physical arrays."""
import unittest
import numpy as np
import torch
from qda_mujoco.dynamics import DynamicPlant
from qda_locomotion.upstream_b2 import UpstreamB2Policy


class ConstantAction(torch.nn.Module):
    def forward(self, obs):
        return torch.arange(12, dtype=torch.float32).reshape(1, 12) / 10.


class B2ContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(2)
        cls.plant = DynamicPlant('/ws/output/v2/dynamics.xml')
        cls.adapter = UpstreamB2Policy('/ws/output/v2/upstream_b2', cls.plant)
        cls.actual_policy = cls.adapter.policy

    def setUp(self):
        self.plant.reset()
        self.adapter.previous[:] = 0.
        self.adapter.policy = self.actual_policy

    def test_named_encoder_permutation(self):
        p, a = self.plant, self.adapter
        p.data.qpos[p.qadr[a.indices]] = a.q0
        p.data.qvel[:] = 0.
        # Independently selected physical joints must land in upstream FR/FL/RR/RL slots.
        p.data.qpos[p.model.jnt_qposadr[p.model.joint('FR_calf_joint').id]] += .3
        p.data.qvel[p.model.jnt_dofadr[p.model.joint('RL_thigh_joint').id]] = 2.
        obs = a.observation(p, [.1, -.2, .3])
        np.testing.assert_allclose(obs[6:9], [.1, -.2, .3])
        expected = np.zeros(12); expected[2] = .3
        np.testing.assert_allclose(obs[9:21], expected, atol=1e-6)
        expected = np.zeros(12); expected[10] = .1
        np.testing.assert_allclose(obs[21:33], expected, atol=1e-6)

    def test_action_permutation_and_arm_isolation(self):
        p, a = self.plant, self.adapter
        a.policy = ConstantAction()
        target = a.target(p, [0., 0., 0.])
        self.assertAlmostEqual(target[p.names.index('FR_calf_joint')], -1.45, places=6)
        self.assertAlmostEqual(target[p.names.index('RL_hip_joint')], .1125, places=6)
        np.testing.assert_array_equal(target[p.arm_indices], p.home[p.arm_indices])
        np.testing.assert_allclose(a.previous, np.arange(12)/10., atol=1e-6)

    def test_original_checkpoint_and_invalid_command(self):
        target = self.adapter.target(self.plant, [0., 0., 0.])
        self.assertEqual(target.shape, self.plant.home.shape)
        self.assertTrue(np.isfinite(target).all())
        with self.assertRaises(ValueError):
            self.adapter.target(self.plant, [float('nan'), 0., 0.])


if __name__ == '__main__':
    unittest.main()
