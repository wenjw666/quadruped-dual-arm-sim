"""Contract, replay and IBVS geometry regressions independent of ROS services."""
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from qda_brain.contracts import parse_local,validate,Journal
from qda_brain.ibvs import command


class Contracts(unittest.TestCase):
    def test_commands_and_ambiguity(self):
        self.assertEqual(parse_local('导航到北侧')['target_id'],'north_entry')
        self.assertEqual(parse_local('右臂回位')['arm'],'right')
        for text in ('不要去北侧','去北侧然后去南侧','左臂和右臂回位','去北侧和南侧','对准','随便抓东西'):
            with self.assertRaises(ValueError):parse_local(text)
        with self.assertRaises(ValueError):validate(dict(skill='shell',object_id='none',arm='none',target_id='none'))

    def test_durable_idempotence(self):
        with tempfile.TemporaryDirectory() as d:
            p=d+'/journal.sqlite';j=Journal(p);g=parse_local('查询状态')
            self.assertFalse(j.reserve('a',g)['duplicate']);self.assertTrue(j.reserve('a',g)['duplicate'])
            with self.assertRaisesRegex(ValueError,'TASK_BUSY'):j.reserve('b',g)
            with self.assertRaisesRegex(ValueError,'TASK_ID_CONFLICT'):j.reserve('a',parse_local('停止'))
            j.db.close();j=Journal(p)
            self.assertEqual(j.get('a')[0]['status'],'INTERRUPTED')
            self.assertFalse(j.reserve('b',g)['duplicate']);j.db.close()

    def test_ibvs_pixel_feedback_converges(self):
        k=np.array([[250.,0,160],[0,250,120],[0,0,1]])
        points=np.array([[-.04,-.04,0],[.04,-.04,0],[.04,.04,0],[-.04,.04,0]])
        points=points@Rotation.from_euler('xyz',[.05,-.04,.07]).as_matrix().T+[.025,-.018,.36]
        errors=[]
        for _ in range(250):
            uv=points[:,:2]/points[:,2,None]*[250,250]+[160,120]
            v,e,_=command(uv,points[:,2],k);errors.append(e)
            # Rigid stationary target as seen from a moving optical camera.
            points=(points-v[:3]*.05)@Rotation.from_rotvec(v[3:]*.05).as_matrix()
        self.assertLess(errors[-1],.1)
        self.assertLess(errors[-1],errors[0]/50)

    def test_ibvs_invalid_measurements(self):
        k=np.eye(3)
        for z in ([0]*4,[float('nan')]*4,[2]*4):
            with self.assertRaises(ValueError):command(np.zeros((4,2)),z,k)
        with self.assertRaises(ValueError):command(np.zeros((4,2)),[.3]*4,k)


if __name__=='__main__':unittest.main(verbosity=2)
