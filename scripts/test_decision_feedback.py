"""Fail-closed semantic review and feedback-change regressions; no ROS needed."""
import unittest
from qda_brain.feedback_review import expected_decision, review_gate, validate_review


class FeedbackReview(unittest.TestCase):
    def setUp(self):
        self.context=dict(fault='',previous_status='NONE',sensors_fresh=True,robot_ready=True,
            active_task=None,all_steps_done=False,candidate='navigate',
            candidate_proposal=dict(skill='navigate',target_id='south_entry',object_id='none',arm='none'))

    def test_pending_model_complete_cannot_dispatch(self):
        self.assertFalse(review_gate(self.context,self.context,{'decision':'complete'})['execution_allowed'])

    def test_bridge_must_match_operator_candidate(self):
        m=dict(decision='dispatch',decision_source='interpretation_bridge',bridge_matches_candidate=True,
            bridge_proposal={**self.context['candidate_proposal'],'target_id':'north_entry'})
        self.assertFalse(review_gate(self.context,self.context,m)['execution_allowed'])
        m['bridge_proposal']=self.context['candidate_proposal']
        self.assertTrue(review_gate(self.context,self.context,m)['execution_allowed'])

    def test_new_fault_or_stale_feedback_cancels_permission(self):
        for change in (dict(fault='tilt'),dict(sensors_fresh=False),dict(active_task='other'),dict(robot_ready=False)):
            self.assertFalse(review_gate(self.context,{**self.context,**change},{'decision':'dispatch'})['execution_allowed'])

    def test_completion_requires_success_and_never_dispatches(self):
        done={**self.context,'candidate':None,'candidate_proposal':None,'all_steps_done':True,'previous_status':'SUCCEEDED'}
        gate=review_gate(done,done,{'decision':'complete'})
        self.assertTrue(gate['gate_passed']);self.assertFalse(gate['execution_allowed'])
        for state in ('NONE','RUNNING','FAILED','MISSING'):
            changed={**done,'previous_status':state}
            self.assertNotEqual(expected_decision(changed),'complete')
            self.assertFalse(review_gate(changed,changed,{'decision':'complete'})['gate_passed'])

    def test_strict_model_output(self):
        for value in ({'decision':'invent'},{'decision':'dispatch','execute':True},None):
            with self.assertRaises(ValueError):validate_review(value)


if __name__=='__main__':unittest.main(verbosity=2)
