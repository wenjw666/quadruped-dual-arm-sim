"""Geometry, colour preservation and bounded fusion contracts."""
import tempfile,unittest
from pathlib import Path
import numpy as np
from qda_perception.color_map import ColorMap

class FusionTests(unittest.TestCase):
    def test_voxel_fusion_preserves_color(self):
        m=ColorMap(.1);m.add(np.array([[.01,.01,.01],[.03,.01,.01],[1.,1.,1.]]),np.array([[255,0,0],[255,0,0],[0,0,255]]))
        x,c=m.arrays();self.assertEqual(len(x),2);np.testing.assert_allclose(x[0],[.02,.01,.01]);np.testing.assert_array_equal(c,[[255,0,0],[0,0,255]])
    def test_capacity_and_nan(self):
        m=ColorMap(.1,1);m.add(np.array([[0,0,0],[1,1,1],[np.nan,0,0]]),np.ones((3,3))*100)
        self.assertEqual(len(m.cells),1);self.assertEqual(m.rejected,1)
    def test_binary_ply_export_matches_observations(self):
        m=ColorMap();m.add(np.array([[1.,2.,3.]]),np.array([[42,80,140]]))
        with tempfile.TemporaryDirectory() as d:
            r=m.save(d);a=Path(d,'colored_map.ply').read_bytes();payload=a.split(b'end_header\n')[1]
            self.assertEqual(r['points'],1);self.assertEqual(len(payload),15);self.assertEqual(payload[-3:],bytes([42,80,140]))
            packed=Path(d,'colored_map.pcd').read_bytes().split(b'DATA binary\n')[1]
            self.assertEqual(len(packed),16);self.assertEqual(np.frombuffer(packed,dtype='<u4')[-1],(42<<16)|(80<<8)|140)
    def test_empty_map_rejected(self):
        with tempfile.TemporaryDirectory() as d:self.assertRaises(ValueError,ColorMap().save,d)

if __name__=='__main__':unittest.main(verbosity=2)
