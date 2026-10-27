"""Auditable checkpoint review, not a free-form autonomous task planner."""
DECISIONS=('dispatch','wait','stop','complete')
REVIEW_SCHEMA={'type':'object','additionalProperties':False,'required':['decision'],
               'properties':{'decision':{'type':'string','enum':list(DECISIONS)}}}

def expected_decision(context):
    if context['fault'] or context['previous_status'] in ('FAILED','CANCELLED','INTERRUPTED','MISSING'):
        return 'stop'
    if not context['sensors_fresh'] or not context['robot_ready'] or context['active_task'] or context['previous_status'] in ('RUNNING','RESERVED'):
        return 'wait'
    if context['all_steps_done']:
        return 'complete' if context['previous_status']=='SUCCEEDED' else 'stop'
    return 'dispatch' if context['candidate'] is not None else 'stop'

def validate_review(value):
    if not isinstance(value,dict) or set(value)!= {'decision'} or value['decision'] not in DECISIONS:
        raise ValueError('INVALID_REVIEW_OUTPUT')
    return value


def review_gate(context,current,model):
    """A semantic mismatch or changed feedback never grants a receipt."""
    expected=expected_decision(context)
    current_expected=expected_decision(current)
    agrees=(model['decision']==expected and current_expected==expected)
    if model.get('decision_source')=='interpretation_bridge':
        agrees=agrees and expected=='dispatch' and model.get('bridge_proposal')==context.get('candidate_proposal')
    return dict(guard_expected=expected,guard_current=current_expected,gate_passed=bool(agrees),
                execution_allowed=bool(agrees and expected=='dispatch'),supervisor_override=False)
