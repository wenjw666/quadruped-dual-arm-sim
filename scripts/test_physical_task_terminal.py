"""Late cleanup cannot erase completion or the original physical failure."""
import sys,unittest
from types import SimpleNamespace
sys.path[:0]=['/ws/src/qda_mujoco']
from qda_mujoco.manipulation import PhysicalManipulation

class TerminalState(unittest.TestCase):
    def state(self,stage,error='',carrying=False):
        state=PhysicalManipulation.__new__(PhysicalManipulation)
        state.stage=stage;state.error=error;state.carrying=carrying
        state.active=False;state.completed=not bool(error)
        state.events=[];state.queue=[];state.d=SimpleNamespace(time=10.)
        return state

    def test_late_cancel_keeps_successful_placement(self):
        state=self.state('placed');state.cancel()
        self.assertEqual((state.stage,state.completed,state.error),('placed',True,''))

    def test_cleanup_keeps_original_failure(self):
        state=self.state('failed','PAYLOAD_LOST');state.cancel()
        self.assertEqual(state.error,'PAYLOAD_LOST')

    def test_cancellation_preserves_payload_ownership(self):
        state=self.state('carrying',carrying=True);state.cancel()
        self.assertTrue(state.carrying);self.assertFalse(state.completed)
        self.assertEqual(state.error,'CANCELLED')

if __name__=='__main__':unittest.main(verbosity=2)
