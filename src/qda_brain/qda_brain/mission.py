"""Bounded sequential plans; every step uses the existing RobotSkill contract."""
import hashlib
from .contracts import validate, SCENE_VERSION


def validate_plan(data):
    if not isinstance(data, dict) or set(data) != {'scene_version', 'steps'}:
        raise ValueError('INVALID_PLAN_FIELDS')
    if data['scene_version'] != SCENE_VERSION:
        raise ValueError('STALE_SCENE')
    if not isinstance(data['steps'], list) or not 1 <= len(data['steps']) <= 12:
        raise ValueError('PLAN_REQUIRES_1_TO_12_STEPS')
    steps = [validate(step) for step in data['steps']]
    # Validate the whole plan before any physical step can start.
    if any(step['skill']=='pause' for step in steps):
        raise ValueError('PLAN_CONTAINS_UNAVAILABLE_OR_CONTROL_SKILL')
    if any(step['skill']=='pick_place' and step['arm']!='left' for step in steps):
        raise ValueError('RIGHT_ARM_PHYSICAL_TASK_NOT_VALIDATED')
    return dict(scene_version=SCENE_VERSION, steps=steps)


def child_id(mission_id, index):
    return 'mission.'+hashlib.sha256(mission_id.encode()).hexdigest()[:24]+'.'+str(index)
