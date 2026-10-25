"""Estimator independence and calibration provenance regression checks."""
import unittest
import mujoco
import numpy as np
from qda_locomotion.transfer_env import B2TransferEnv
from qda_locomotion.velocity_adapter import ContactVelocityEstimator,VelocityAdapter


class VelocityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env=B2TransferEnv(randomize=False)

    def setUp(self):
        self.env.reset(seed=7,options={'command':[0,0,0]})

    def test_estimator_does_not_read_free_base_linear_velocity(self):
        p=self.env.plant
        p.data.qvel[p.dadr[p.leg_indices]]=.01
        mujoco.mj_forward(p.model,p.data)
        a=ContactVelocityEstimator(p).update(p,np.zeros(3),np.ones(4))[0]
        p.data.qvel[:3]=[9.,-6.,3.]
        mujoco.mj_forward(p.model,p.data)
        b=ContactVelocityEstimator(p).update(p,np.zeros(3),np.ones(4))[0]
        np.testing.assert_allclose(a,b,atol=1e-10)

    def test_missing_contact_is_invalid(self):
        _,valid,disagreement=ContactVelocityEstimator(self.env.plant).update(self.env.plant,np.zeros(3),np.zeros(4))
        self.assertFalse(valid);self.assertTrue(np.isinf(disagreement))

    def test_contract_and_bounded_stop(self):
        p=self.env.plant;file='/ws/output/v2_audit/b2_command_response.json'
        with self.assertRaises(ValueError):VelocityAdapter(p,file,'different_model')
        a=VelocityAdapter(p,file,self.env.contract['model_sha256'])
        for _ in range(30):value,_=a.command([.15,0,0],p)
        self.assertTrue(np.isfinite(value).all());self.assertLessEqual(abs(value[0]),.6)
        for _ in range(30):value,_=a.command([0,0,0],p)
        np.testing.assert_array_equal(value,np.zeros(3))


if __name__=='__main__':unittest.main()
