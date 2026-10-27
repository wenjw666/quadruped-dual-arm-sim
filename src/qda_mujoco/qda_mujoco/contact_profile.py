"""Explicit numerical contact profile for the free-object manipulation task.

The locomotion actor is still experimental. These settings are reported separately
from its original training contract; they are not a claim of retraining.
"""
import mujoco

PROFILE = 'robotiq_contact_v1'

def repair_gripper_mimics(model):
    changed = []
    for i in range(model.neq):
        if model.eq_type[i] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        name = model.joint(int(model.eq_obj1id[i])).name
        if '_right_finger_tip_joint' in name or '_right_inner_knuckle_joint' in name:
            expected = model.joint(name.split('_')[0] + '_finger_right_joint').id
            if model.eq_obj2id[i] != expected:
                model.eq_obj2id[i] = expected
                changed.append(name)
    return changed

def apply_contact_profile(model):
    repairs = repair_gripper_mimics(model)
    model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    model.opt.noslip_iterations = 0
    model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    gid = model.geom('pick_object_collision_0').id
    model.geom_priority[gid] = 1
    model.geom_solref[gid] = [.004, 1.]
    model.geom_solimp[gid] = [.95, .99, .001, .5, 2.]
    model.geom_condim[gid] = 6
    model.geom_friction[gid] = [.9, .01, .001]
    return dict(name=PROFILE, mimic_repairs=repairs, multiccd=True,
                cone='elliptic', noslip_iterations=0, object_condim=6, object_friction=[.9,.01,.001],
                object_solref=[.004,1.], object_solimp=[.95,.99,.001,.5,2.],
                calibrated_hardware=False, policy_retrained=False)
