"""The factory deployment must keep the tested robot and reject physics drift."""
import unittest
import os
import mujoco
from qda_mujoco.scene_control import robot_physics_signature,SceneController


class SceneContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller=SceneController(os.environ.get('QDA_SCENE_CONTRACT_PATH','/ws/output/v2_scene/factory_dynamic.xml'),'/ws/output/v2/dynamics.xml',
            '/ws/output/v2/runs/b2_transfer_v1/initial_policy.onnx','/ws/output/v2_audit/b2_command_response.json')

    def test_robot_equal_despite_world_and_camera_differences(self):
        reference=mujoco.MjModel.from_xml_path('/ws/output/v2/dynamics.xml')
        scene=self.controller.plant.model
        self.assertGreater(scene.nbody,reference.nbody)
        self.assertGreater(scene.ncam,reference.ncam)
        self.assertNotEqual(robot_physics_signature(scene),robot_physics_signature(reference))
        from qda_mujoco.contact_profile import repair_gripper_mimics
        repair_gripper_mimics(reference)
        reference.opt.cone=scene.opt.cone;reference.opt.enableflags=scene.opt.enableflags
        reference.opt.noslip_iterations=scene.opt.noslip_iterations
        self.assertEqual(robot_physics_signature(scene),robot_physics_signature(reference))
        self.assertFalse(self.controller.binding['contact_profile']['policy_retrained'])

    def test_solver_changes_are_detected(self):
        model=self.controller.plant.model;original=robot_physics_signature(model)
        for field in ('cone','enableflags','disableflags','noslip_iterations'):
            value=getattr(model.opt,field)
            try:
                setattr(model.opt,field,0 if value else 1)
                self.assertNotEqual(robot_physics_signature(model),original)
            finally:setattr(model.opt,field,value)

    def test_mass_and_gravity_changes_are_detected(self):
        model=self.controller.plant.model;original=robot_physics_signature(model)
        index=model.body('base_link').id;mass=model.body_mass[index]
        try:
            model.body_mass[index]+=1.
            self.assertNotEqual(robot_physics_signature(model),original)
        finally:model.body_mass[index]=mass
        gravity=model.opt.gravity.copy()
        try:
            model.opt.gravity[2]=0.
            self.assertNotEqual(robot_physics_signature(model),original)
        finally:model.opt.gravity[:]=gravity


if __name__=='__main__':unittest.main()
