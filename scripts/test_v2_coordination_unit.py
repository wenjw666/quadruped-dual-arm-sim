"""Ownership, plan preflight and measured-image tracking regressions."""
import unittest
import cv2
import numpy as np
from qda_bringup.control_lease import BaseLease
from qda_brain.contracts import SCENE_VERSION, parse_local
from qda_brain.mission import validate_plan, child_id
from qda_brain.feature_tracker import MarkerTracker


class Coordination(unittest.TestCase):
    def test_lease_token_fencing_expiry_and_reset(self):
        lease=BaseLease();first=lease.request({'op':'acquire','owner':'task-a'},0)
        with self.assertRaisesRegex(ValueError,'BUSY'):lease.request({'op':'acquire','owner':'task-b'},1)
        with self.assertRaisesRegex(ValueError,'STALE'):lease.request({'op':'mode','token':'wrong','mode':'nav'},1)
        lease.request({'op':'mode','token':first['token'],'mode':'nav'},1)
        lease.request({'op':'mode','token':first['token'],'mode':'task'},1)
        self.assertEqual(lease.current['mode'],'task')
        self.assertFalse(lease.expire(3.9));self.assertTrue(lease.expire(4.))
        with self.assertRaisesRegex(ValueError,'STALE'):lease.request({'op':'renew','token':first['token']},4.1)
        with self.assertRaisesRegex(ValueError,'RESET_REQUIRED'):lease.request({'op':'acquire','owner':'task-b'},4.1)
        lease.request({'op':'reset'},4.2);second=lease.request({'op':'acquire','owner':'task-b'},4.2)
        self.assertNotEqual(first['token'],second['token'])
        with self.assertRaisesRegex(ValueError,'STALE'):lease.request({'op':'release','token':first['token']},4.3)
        self.assertEqual(lease.current['owner'],'task-b')

    def test_plan_validates_all_steps_before_execution(self):
        plan=dict(scene_version=SCENE_VERSION,steps=[parse_local('查询状态'),parse_local('左臂回位')])
        self.assertEqual(len(validate_plan(plan)['steps']),2)
        with self.assertRaisesRegex(ValueError,'STALE_SCENE'):validate_plan({**plan,'scene_version':SCENE_VERSION-1})
        pick=dict(skill='pick_place',object_id='red_cube',target_id='place_station',arm='left')
        self.assertEqual(validate_plan({**plan,'steps':[pick]})['steps'][0],pick)
        with self.assertRaisesRegex(ValueError,'RIGHT_ARM_PHYSICAL_TASK_NOT_VALIDATED'):
            validate_plan({**plan,'steps':[{**pick,'arm':'right'}]})
        bad=dict(skill='pause',object_id='none',target_id='none',arm='none')
        with self.assertRaisesRegex(ValueError,'UNAVAILABLE'):validate_plan({**plan,'steps':[plan['steps'][0],bad]})
        with self.assertRaises(ValueError):validate_plan({**plan,'steps':plan['steps']*7})
        self.assertEqual(child_id('id',0),child_id('id',0));self.assertNotEqual(child_id('id',0),child_id('id',1))
        self.assertLessEqual(len(child_id('x'*128,11)),128)

    def test_real_klt_translation_reacquisition_and_finite_bridge(self):
        gray=np.full((240,320),255,np.uint8)
        gray[70:150,110:190]=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),17,80)
        tracker=MarkerTracker(17);points,source,_=tracker.update(gray,1.)
        self.assertEqual(source,'aruco');detector=tracker.detect
        tracker.detect=lambda image:None
        for i in (1,2):
            moved=cv2.warpAffine(gray,np.float32([[1,0,2*i],[0,1,i]]),(320,240),borderValue=255)
            corners,source,misses=tracker.update(moved,1.+i*.2)
            self.assertEqual(source,'klt');self.assertEqual(misses,i)
            self.assertLess(np.max(np.abs(corners-points-[2*i,i])),.2)
        with self.assertRaisesRegex(ValueError,'MARKER_LOST'):tracker.update(moved,1.6)
        tracker.detect=detector
        _,source,misses=tracker.update(moved,1.8);self.assertEqual((source,misses),('aruco',0))
        with self.assertRaisesRegex(ValueError,'TIME_REGRESSION'):tracker.update(gray,1.7)

    def test_klt_rejects_blank_and_wrong_identity(self):
        blank=np.full((240,320),255,np.uint8);gray=blank.copy()
        gray[70:150,110:190]=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),17,80)
        tracker=MarkerTracker(17);tracker.update(gray,1.)
        with self.assertRaises(ValueError):tracker.update(blank,1.2)
        with self.assertRaisesRegex(ValueError,'MARKER_LOST'):MarkerTracker(18).update(gray,1.)


if __name__=='__main__':unittest.main(verbosity=2)
