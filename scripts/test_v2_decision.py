"""Guard/ownership regressions with explicit fake skills, not a robot success claim."""
import unittest
from qda_task.decision import TaskGoal,Observation,CommandArbiter,DecisionExecutor

class Driver:
 def __init__(self):self.starts=[];self.cancels=[];self.holds=0;self.result='SUCCESS'
 def start(self,s,g):self.starts.append(s);return len(self.starts)
 def poll(self,h):return self.result
 def cancel(self,h):self.cancels.append(h)
 def hold(self):self.holds+=1

class Tests(unittest.TestCase):
 def setUp(self):
  self.d=Driver();self.e=DecisionExecutor(self.d,['cube'],['tray'],stable_duration=.1)
  self.g=TaskGoal('task1','cube','tray','left',1);self.e.submit(self.g,0.)
 def obs(self,t,**kw):
  values=dict(stamp=t,scene_version=1,base_speed=0.,base_tilt=0.,object_id='cube',target_id='tray',
              pose_valid=True,confidence=.9,depth_valid=True,held=True,released=True,gripper_empty=True,
              arm_retracted=True,placement_error=.01,visual_confirmation=True,source='vision')
  values.update(kw);return Observation(**values)
 def test_complete_once(self):
  for i in range(12):self.e.tick(self.obs(i*.11),i*.11)
  self.assertEqual(self.e.history['task1']['status'],'SUCCESS')
  self.assertEqual(self.d.starts,list(self.e.STAGES))
  self.assertEqual(self.e.submit(self.g,2.)['status'],'SUCCESS');self.assertEqual(len(self.d.starts),6)
 def test_stale_cancel_running(self):
  self.d.result='RUNNING';self.e.tick(self.obs(0),0)
  self.assertEqual(self.e.tick(self.obs(0),.5)['error'],'STALE_OBSERVATION');self.assertEqual(self.d.cancels,[1])
 def test_no_truth_fallback(self):
  for i in range(6):self.e.tick(self.obs(i*.11,source='configured'),i*.11)
  self.assertEqual(self.d.starts,['navigate'])
 def test_no_repeated_release_when_verification_fails(self):
  for i in range(12):self.e.tick(self.obs(i*.11,placement_error=.021),i*.11)
  self.assertEqual(self.e.history['task1']['status'],'RUNNING');self.assertEqual(self.d.starts.count('place'),1)
  self.assertEqual(self.e.tick(self.obs(40.,placement_error=.021),40.)['error'],'STAGE_TIMEOUT')
 def test_id_conflict_and_busy(self):
  with self.assertRaises(ValueError):self.e.submit(TaskGoal('task1','cube','tray','right',1),0.)
  with self.assertRaises(RuntimeError):self.e.submit(TaskGoal('task2','cube','tray','left',1),0.)
 def test_cancel_is_hold_only(self):
  self.e.cancel('task1',.2);self.assertEqual(self.d.holds,1);self.assertEqual(self.d.starts,[])
 def test_release_waits_for_empty_without_requiring_held(self):
  for i in range(5):self.e.tick(self.obs(i*.11),i*.11)
  self.e.tick(self.obs(.55,released=False,gripper_empty=False),.55)
  self.assertEqual(self.e.history['task1']['stage'],'place')
  self.e.tick(self.obs(.66,held=False),.66)
  self.assertEqual(self.e.history['task1']['stage'],'verify')
 def test_lease_expiry_and_stale_token(self):
  a=CommandArbiter();old=a.acquire('base','nav',0.)
  with self.assertRaises(RuntimeError):a.acquire('base','servo',.1)
  new=a.acquire('base','align',.6);a.release('base',old)
  self.assertFalse(a.valid('base',old,.7));self.assertTrue(a.valid('base',new,.7))
  with self.assertRaises(RuntimeError):a.renew('base',old,.7)
 def test_transfer_allows_motion_but_waits_for_stop(self):
  for i in range(4):self.e.tick(self.obs(i*.11),i*.11)
  self.assertEqual(self.e.history['task1']['stage'],'transfer')
  self.d.result='RUNNING'
  row=self.e.tick(self.obs(.44,base_speed=.15),.44)
  self.assertEqual(row['status'],'RUNNING');self.assertEqual(row['stage'],'transfer')
  self.d.result='SUCCESS'
  self.e.tick(self.obs(.55,base_speed=.10),.55)
  self.assertEqual(self.e.history['task1']['stage'],'transfer')
  self.e.tick(self.obs(.66),.66);self.e.tick(self.obs(.78),.78)
  self.assertEqual(self.e.history['task1']['stage'],'place')
  self.assertEqual(self.d.starts.count('transfer'),1)
 def test_payload_loss_during_transfer_fails(self):
  for i in range(4):self.e.tick(self.obs(i*.11),i*.11)
  row=self.e.tick(self.obs(.44,held=False,base_speed=.15),.44)
  self.assertEqual(row['error'],'PAYLOAD_UNCONFIRMED')

if __name__=='__main__':unittest.main()
