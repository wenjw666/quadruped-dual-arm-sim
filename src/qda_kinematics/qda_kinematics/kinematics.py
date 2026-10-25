"""Analytic kinematics for the quadruped legs and the two 4-DoF arms.

Joint zeros match the URDF:
  legs: hip about X, thigh about Y, knee about Y, all measured from a
  straight-down leg.
  arms: pan about Z, then three pitches about Y, measured from a straight
  arm along +X. A top-down grasp points the tool +X axis at world -Z.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

LEG_ORDER = ("FL", "FR", "RL", "RR")
LEG_JOINTS: List[str] = [
    f"{leg}_{part}_joint" for leg in LEG_ORDER for part in ("hip", "thigh", "calf")
]

# B2 hip joints in the trunk frame. y>0 is the left side.
HIP_XYZ = {
    "FL": np.array([0.3285, 0.072, 0.0]),
    "FR": np.array([0.3285, -0.072, 0.0]),
    "RL": np.array([-0.3285, 0.072, 0.0]),
    "RR": np.array([-0.3285, -0.072, 0.0]),
}

# B2 calf joints only accept a negative bend on every leg.
KNEE_SIGN = {"FL": -1.0, "FR": -1.0, "RL": -1.0, "RR": -1.0}

THIGH_LEN = 0.32
CALF_LEN = 0.35
FOOT_RADIUS = 0.032
STANCE_HEIGHT = 0.50  # hip origin to foot-sphere center

ARM_LINK = (0.16, 0.15, 0.07)  # upper, forearm, tool
# Pointing straight up keeps the arms clear of the tables at spawn.
ARM_HOME = (0.0, -math.pi / 2.0, 0.0, 0.0)
# Photograph-inspired horizontal shoulders; calibrated relaxed display/home poses.
AUBO_HOME_LEFT = (-0.566113381, -0.809603385, -2.419106216, 2.156220402, 0.499311776, 1.017, 1.54)
AUBO_HOME_RIGHT = (0.566111467, -0.809600831, 2.419110778, 2.156225224, -0.49931376, 1.017, -1.54)
# Servo-ready wrists: measured FCL link5/link7 clearance exceeds 13 mm,
# whereas the photographed folded display pose only has a few millimetres.
AUBO_SERVO_LEFT = (*AUBO_HOME_LEFT[:5], .70, 0.)
AUBO_SERVO_RIGHT = (*AUBO_HOME_RIGHT[:5], .70, 0.)
# Calibrated carry pad target (0.29, 0.32, 0.28) in base_link.
AUBO_CARRY_LEFT = (-2.02240215, -2.227231155, -1.592419902, 2.287937532, -0.75840814, 1.007216445, 1.532523706)
AUBO_LIMITS = (
    (-3.105, 3.105),
    (-3.663, 0.523),
    (-3.105, 3.105),
    (-1.047, 2.529),
    (-3.105, 3.105),
    (-1.047, 1.047),
    (-1.57, 1.57),
)
FINGER_OPEN = 0.0  # Robotiq knuckle angle, radians (open)
FINGER_CLOSED = 0.8
# Robotiq linkage: q=0 opens 85 mm; q=0.42 opens approximately 43 mm.
GRASP_FINGER = 0.42  # approximate 40 mm opening, checked on the CAD linkage

LEG_LIMITS = {
    "hip": (-0.80, 0.80),
    "thigh": (-0.90, 2.60),
    "calf": (-2.70, -0.50),
}
ARM_LIMITS = (
    (-1.80, 1.80),
    (-2.40, 1.40),
    (-2.60, 2.60),
    (-2.60, 2.60),
)

CYCLE = 0.56
DUTY = 0.64
SWING_HEIGHT = 0.05
# Diagonal trot: FL+RR swing together, FR+RL half a cycle later.
PHASE_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


def _rot_x(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class LegKinematics:
    def __init__(self, thigh: float = THIGH_LEN, calf: float = CALF_LEN):
        self.thigh = thigh
        self.calf = calf

    def fk(self, qhip: float, qth: float, qkn: float) -> np.ndarray:
        knee = (_rot_x(qhip) @ _rot_y(qth)) @ np.array([0.0, 0.0, -self.thigh])
        foot_dir = (_rot_x(qhip) @ _rot_y(qth) @ _rot_y(qkn)) @ np.array([0.0, 0.0, -self.calf])
        return knee + foot_dir

    def knee_position(self, qhip: float, qth: float, qkn: float) -> np.ndarray:
        del qkn
        return (_rot_x(qhip) @ _rot_y(qth)) @ np.array([0.0, 0.0, -self.thigh])

    def ik(self, x: float, y: float, z: float, knee_sign: float) -> np.ndarray:
        """Foot target in the hip frame. Returns hip, thigh, knee."""
        z_s = -math.hypot(y, z)
        qhip = math.atan2(y, -z)
        reach = math.hypot(x, z_s)
        reach = _clamp(reach, abs(self.thigh - self.calf) + 1e-4, self.thigh + self.calf - 1e-4)
        cos_phi = (self.thigh ** 2 + self.calf ** 2 - reach ** 2) / (2.0 * self.thigh * self.calf)
        phi = math.acos(_clamp(cos_phi, -1.0, 1.0))
        qkn = knee_sign * (math.pi - phi)
        alpha = math.atan2(x, -z_s)
        cos_beta = (self.thigh ** 2 + reach ** 2 - self.calf ** 2) / (2.0 * self.thigh * reach)
        beta = math.acos(_clamp(cos_beta, -1.0, 1.0))

        best_err = 1e9
        best = np.zeros(3)
        target = np.array([x, y, z])
        for qth in (alpha + beta, alpha - beta):
            # Go1's calf joint is negative. Keep the branch that lands in that range.
            for qth_f, qkn_f in ((qth, qkn), (-qth, -qkn)):
                if qkn_f > -0.2:
                    continue
                foot = self.fk(qhip, qth_f, qkn_f)
                err = float(np.linalg.norm(foot - target))
                if err < best_err:
                    best_err = err
                    best = np.array([qhip, qth_f, qkn_f])
        if best_err > 0.015:
            raise RuntimeError(f"leg IK failed, error {best_err:.4f} for {(x, y, z)}")
        return best


class ArmKinematics:
    def __init__(self, links: Sequence[float] = ARM_LINK):
        self.l1, self.l2, self.l3 = links

    def fk(self, q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        rot = _rot_z(q[0])
        point = np.zeros(3)
        rot = rot @ _rot_y(q[1])
        point = point + rot @ np.array([self.l1, 0.0, 0.0])
        rot = rot @ _rot_y(q[2])
        point = point + rot @ np.array([self.l2, 0.0, 0.0])
        rot = rot @ _rot_y(q[3])
        point = point + rot @ np.array([self.l3, 0.0, 0.0])
        approach = rot @ np.array([1.0, 0.0, 0.0])
        return point, approach

    def ik(self, x: float, y: float, z: float, seed: Optional[Sequence[float]] = None) -> np.ndarray:
        """Tool-tip position for a gripper pointing straight down."""
        wrist_x, wrist_y, wrist_z = x, y, z + self.l3
        q1 = math.atan2(wrist_y, wrist_x)
        c, s = math.cos(q1), math.sin(q1)
        planar_x = c * wrist_x + s * wrist_y
        planar_y = -s * wrist_x + c * wrist_y
        if abs(planar_y) > 1e-3:
            raise RuntimeError("arm target is outside the pan plane")
        reach = math.hypot(planar_x, wrist_z)
        cos3 = (reach ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2.0 * self.l1 * self.l2)
        if cos3 < -1.0 or cos3 > 1.0:
            raise RuntimeError(f"arm target out of reach: {(x, y, z)}")
        q3_mag = math.acos(_clamp(cos3, -1.0, 1.0))
        solutions = []
        for q3 in (q3_mag, -q3_mag):
            q2 = math.atan2(-wrist_z, planar_x) - math.atan2(
                self.l2 * math.sin(q3), self.l1 + self.l2 * math.cos(q3)
            )
            q4 = math.pi / 2.0 - (q2 + q3)
            q = np.array([q1, q2, q3, q4])
            if not self.within_limits(q):
                continue
            tip, approach = self.fk(q)
            err = float(np.linalg.norm(tip - np.array([x, y, z])))
            if err > 1e-4 or approach[2] > -0.98:
                continue
            if seed is None:
                score = 0.0
            else:
                score = float(np.linalg.norm(q - np.asarray(seed, dtype=float)))
            # Prefer the elbow-up branch when seeds are equal.
            elbow = self.elbow_height(q)
            solutions.append((score, -elbow, q))
        if not solutions:
            raise RuntimeError(f"no limited arm IK for {(x, y, z)}")
        solutions.sort(key=lambda item: (item[0], item[1]))
        return solutions[0][2]

    def elbow_height(self, q: Sequence[float]) -> float:
        rot = _rot_z(q[0]) @ _rot_y(q[1])
        return float((rot @ np.array([self.l1, 0.0, 0.0]))[2])

    @staticmethod
    def within_limits(q: Sequence[float]) -> bool:
        return all(lo <= value <= hi for value, (lo, hi) in zip(q, ARM_LIMITS))


def _clip_leg(q: np.ndarray) -> np.ndarray:
    out = np.array(q, dtype=float)
    for index, part in enumerate(("hip", "thigh", "calf")):
        lo, hi = LEG_LIMITS[part]
        out[index] = _clamp(float(out[index]), lo, hi)
    return out


def stance_command(legs: Optional[LegKinematics] = None) -> np.ndarray:
    """Absolute 12-vector in LEG_JOINTS order for the nominal stance."""
    solver = legs or LegKinematics()
    command = []
    for name in LEG_ORDER:
        q = solver.ik(0.0, 0.0, -STANCE_HEIGHT, KNEE_SIGN[name])
        command.extend(_clip_leg(q).tolist())
    return np.asarray(command, dtype=float)


def foot_targets(
    phase: float,
    velocity: Sequence[float],
    measured: Sequence[float],
) -> Dict[str, np.ndarray]:
    """Raibert trot. velocity/measured are base-frame vx, vy, yaw rate.

    A near-zero command holds the stance instead of marching in place.
    """
    vx, vy, wz = (float(velocity[0]), float(velocity[1]), float(velocity[2]))
    mvx, mvy = float(measured[0]), float(measured[1])
    moving = abs(vx) > 0.005 or abs(vy) > 0.005 or abs(wz) > 0.02
    targets = {}
    t_stance = DUTY * CYCLE
    for name in LEG_ORDER:
        hx, hy, _hz = HIP_XYZ[name]
        # Half-step feedforward plus a small tracking term on planar velocity.
        x_off = 0.5 * vx * t_stance + 0.10 * (vx - mvx)
        y_off = 0.5 * vy * t_stance + 0.10 * (vy - mvy)
        x_off += 0.5 * (-wz * hy) * t_stance
        y_off += 0.5 * (wz * hx) * t_stance
        x_off = _clamp(x_off, -0.07, 0.07)
        y_off = _clamp(y_off, -0.05, 0.05)
        if not moving:
            targets[name] = np.array([0.0, 0.0, -STANCE_HEIGHT])
            continue
        local = (phase + PHASE_OFFSET[name]) % 1.0
        if local < DUTY:
            tau = local / DUTY
            x = x_off * (1.0 - 2.0 * tau)
            y = y_off * (1.0 - 2.0 * tau)
            z = -STANCE_HEIGHT
        else:
            tau = (local - DUTY) / (1.0 - DUTY)
            x = (-x_off) + (2.0 * x_off) * tau
            y = (-y_off) + (2.0 * y_off) * tau
            z = -STANCE_HEIGHT + SWING_HEIGHT * math.sin(math.pi * tau)
        targets[name] = np.array([x, y, z])
    return targets


def joint_command(
    phase: float,
    velocity: Sequence[float],
    measured: Sequence[float],
    legs: Optional[LegKinematics] = None,
) -> np.ndarray:
    """Go1 trot around the nominal stance.

    The calf joint cannot cross zero, so the swing is a bounded pitch
    added to the stance instead of a free-space IK solution.
    """
    del measured
    command = stance_command(legs)
    vx, vy, wz = (float(velocity[0]), float(velocity[1]), float(velocity[2]))
    amplitude = min(1.0, max(abs(vx) / 0.16, abs(vy) / 0.10, abs(wz) / 0.45))
    if amplitude < 0.01:
        return command
    for index, name in enumerate(LEG_ORDER):
        hx, hy, _ = HIP_XYZ[name]
        forward = float(np.clip((vx - wz * hy) / 0.16, -1.0, 1.0))
        sideways = float(np.clip((vy + wz * hx) / 0.10, -1.0, 1.0))
        local = (phase + PHASE_OFFSET[name]) % 1.0
        if local < DUTY:
            tau = local / DUTY
            stride = math.cos(math.pi * tau)
            lift = 0.0
        else:
            tau = (local - DUTY) / (1.0 - DUTY)
            stride = -math.cos(math.pi * tau)
            lift = math.sin(math.pi * tau) ** 2
        # Continuous position and velocity at lift-off/touch-down; scale the
        # same diagonal gait with speed and reverse the stride when backing up.
        command[3 * index] += 0.10 * sideways * stride
        command[3 * index + 1] += 0.10 * forward * stride + 0.22 * amplitude * lift
        command[3 * index + 2] -= 0.22 * amplitude * lift
        command[3 * index : 3 * index + 3] = _clip_leg(command[3 * index : 3 * index + 3])
    return command


# Finger-pad center, measured along the gripper +Z from the gripper link origin.
from .assembly import GRIPPER_PAD, GRIPPER_ADAPTER, LEFT_ARM_BASE, RIGHT_ARM_BASE, LEFT_ARM_RPY, RIGHT_ARM_RPY
GRASP_PAD = GRIPPER_PAD
_AUBO_XYZ = np.array(
    [
        [0.0, 0.0, 0.1685],
        [0.0, 0.0, 0.0],
        [0.0, 0.225, 0.0],
        [-0.018, 0.0, 0.0],
        [0.018, 0.2678, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ],
    dtype=float,
)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


_AUBO_FIXED = [
    _rpy_matrix(*angles)
    for angles in (
        (0.0, 0.0, 0.0),
        (1.5708, -1.5708, 0.0),
        (-1.5708, 0.0, 0.0),
        (1.5708, 0.0, 0.0),
        (-1.5708, 0.0, 0.0),
        (1.5708, -1.5708, 0.0),
        (1.5708, -1.5708, 0.0),
    )
]
_FLANGE_R = _rpy_matrix(1.5707963, 0.0, 0.0)
_LEFT_MOUNT = _rpy_matrix(*LEFT_ARM_RPY)
_RIGHT_MOUNT = _rpy_matrix(*RIGHT_ARM_RPY)
_AUBO_LO = np.array([lo + 0.03 for lo, _hi in AUBO_LIMITS], dtype=float)
_AUBO_HI = np.array([hi - 0.03 for _lo, hi in AUBO_LIMITS], dtype=float)


def _aubo_fk(q: np.ndarray, mount: np.ndarray, base: np.ndarray):
    """Batch FK. q is (N, 7). Returns pad (N, 3), tool (N, 3), samples (N, 9, 3)."""
    n = q.shape[0]
    pos = np.tile(base, (n, 1))
    rot = np.broadcast_to(mount, (n, 3, 3)).copy()
    samples = []
    for index in range(7):
        ang = q[:, index]
        c, s = np.cos(ang), np.sin(ang)
        rz = np.zeros((n, 3, 3))
        rz[:, 0, 0] = c
        rz[:, 0, 1] = -s
        rz[:, 1, 0] = s
        rz[:, 1, 1] = c
        rz[:, 2, 2] = 1.0
        local = np.einsum("ij,njk->nik", _AUBO_FIXED[index], rz)
        pos = pos + np.einsum("nij,j->ni", rot, _AUBO_XYZ[index])
        rot = np.einsum("nij,njk->nik", rot, local)
        samples.append(pos.copy())
    pos = pos + np.einsum("nij,j->ni", rot, np.array([0.0, -0.136, 0.0]))
    rot = np.einsum("nij,jk->nik", rot, _FLANGE_R)
    origin = pos + GRIPPER_ADAPTER * rot[:, :, 2]
    tool = rot[:, :, 2]
    pad = origin + GRASP_PAD * tool
    palm = origin + 0.04 * tool
    samples.extend((origin, palm))
    return pad, tool, np.stack(samples, axis=1)


def _blocked(samples: np.ndarray, target: np.ndarray, right: np.ndarray) -> np.ndarray:
    """True where a pose dips into the bench, the deck, or the parked right arm."""
    n = samples.shape[0]
    flat = samples.reshape(-1, 3)
    deck_c = np.array([0.02, 0.0, 0.125])
    deck_h = np.array([0.28, 0.21, 0.025])
    in_deck = np.all(np.abs(flat - deck_c) <= deck_h, axis=1).reshape(n, -1).any(axis=1)
    # Table top sits near z=-0.01 in base_link at the normal stance height.
    # Reject links that go through the bench, not links that pass above it.
    below = (samples[:, :, 2] < -0.02) & (samples[:, :, 0] > target[0] - 0.28)
    in_table = below.any(axis=1)
    mast = samples[:, :, :2] - np.array([LEFT_ARM_BASE[0], 0.0])
    near_mast = (np.linalg.norm(mast, axis=2) < 0.07) & (samples[:, :, 2] > 0.15) & (samples[:, :, 2] < 0.54)
    gap = samples[:, :, None, :] - right[None, None, :, :]
    near_right = np.linalg.norm(gap, axis=-1).min(axis=(1, 2)) < 0.09
    return in_deck | in_table | near_mast.any(axis=1) | near_right


def left_grasp_joints(target, count: int = 3, seed=None) -> List[np.ndarray]:
    """Top-down left-arm joints that put the finger pad on ``target`` in base_link.

    Pass ``seed`` to stay on the same joint branch. Hover, grasp and lift must
    share one branch, otherwise the wrist swings sideways and knocks the cube.
    """
    point = np.asarray(target, dtype=float)
    origin = None if seed is None else np.asarray(seed, dtype=float)
    rng = np.random.default_rng(0 if origin is None else 1)
    home = np.array(AUBO_HOME_RIGHT, dtype=float)[None, :]
    _pad_r, _tool_r, right = _aubo_fk(home, _RIGHT_MOUNT, np.array(RIGHT_ARM_BASE))
    right_pts = right[0]
    # Straight down is preferred. A grasp up to about 33 deg off vertical is
    # still accepted, so a few centimetres of navigation error stays solvable.
    prefer_down = -0.98
    accept_down = -0.84 if origin is None else -0.80

    def score_of(joints: np.ndarray):
        pad, tool, samples = _aubo_fk(joints, _LEFT_MOUNT, np.array(LEFT_ARM_BASE))
        err = np.linalg.norm(pad - point, axis=1)
        down = np.clip(tool[:, 2] - prefer_down, 0.0, None)
        bad = _blocked(samples, point, right_pts) | (tool[:, 2] > accept_down + 0.04)
        drift = 0.0
        if origin is not None:
            drift = 0.04 * np.linalg.norm(joints - origin, axis=1)
            bad = bad | (np.linalg.norm(joints - origin, axis=1) > 1.1)
        return err + 0.80 * down + drift + np.where(bad, 10.0, 0.0), err, tool[:, 2], bad

    if origin is None:
        q = rng.uniform(_AUBO_LO, _AUBO_HI, size=(24000, 7))
        cost, _, _, _ = score_of(q)
        # Keep a wider set of wrist/elbow branches for mesh collision checks
        # performed by MoveIt. Point-sample screening cannot see wrist shells.
        order = np.argsort(cost)[:48]
        seeds = q[order]
        base_cost = cost[order]
    else:
        seeds = np.clip(origin + rng.normal(0.0, 0.015, size=(8, 7)), _AUBO_LO, _AUBO_HI)
        base_cost, _, _, _ = score_of(seeds)
    rounds = 6 if origin is None else 8
    sigma = 0.07 if origin is None else 0.035
    width = 48 if origin is None else 64
    for _ in range(rounds):
        noise = rng.normal(0.0, sigma, size=(seeds.shape[0], width, 7))
        trial = np.clip(seeds[:, None, :] + noise, _AUBO_LO, _AUBO_HI).reshape(-1, 7)
        trial_cost, _, _, _ = score_of(trial)
        trial_cost = trial_cost.reshape(seeds.shape[0], width)
        best = trial_cost.argmin(axis=1)
        chosen = trial.reshape(seeds.shape[0], width, 7)[np.arange(seeds.shape[0]), best]
        improved = trial_cost.min(axis=1) < base_cost
        seeds = np.where(improved[:, None], chosen, seeds)
        base_cost = np.where(improved, trial_cost.min(axis=1), base_cost)
    final_cost, final_err, final_down, final_bad = score_of(seeds)
    ranked = np.argsort(final_cost)
    err_limit = 0.015 if origin is None else 0.02
    picked: List[np.ndarray] = []
    for index in ranked:
        if final_bad[index] or final_err[index] > err_limit or final_down[index] > accept_down + 0.02:
            continue
        joints = seeds[index]
        if any(float(np.linalg.norm(joints - kept)) < 0.7 for kept in picked):
            continue
        picked.append(joints.copy())
        if len(picked) >= count:
            break
    return picked


def self_check() -> None:
    legs = LegKinematics()
    rng = np.random.default_rng(1)
    for _ in range(40):
        x = 0.0
        y = float(rng.uniform(-0.02, 0.02))
        z = float(rng.uniform(-0.54, -0.46))
        for sign in (-1.0, 1.0):
            q = legs.ik(x, y, z, sign)
            err = np.linalg.norm(legs.fk(*q) - np.array([x, y, z]))
            if err > 0.02:
                raise AssertionError(f"leg IK error {err}")
    stance = stance_command(legs)
    if stance.shape != (12,):
        raise AssertionError(stance.shape)
    # Front knees point forward (+X), rear knees point backward.
    for index, name in enumerate(LEG_ORDER):
        q = stance[3 * index : 3 * index + 3]
        knee = legs.knee_position(*q)
        if not (-2.70 <= q[2] <= -0.50):
            raise AssertionError(f"{name} calf {q[2]} is outside the stance range")

    arm = ArmKinematics()
    home_tip, home_axis = arm.fk(ARM_HOME)
    if home_tip[2] < 0.30 or abs(home_tip[0]) > 0.05:
        raise AssertionError(f"home pose is not tucked up: {home_tip}")
    for target in (
        (0.26, 0.0, -0.15),
        (0.22, 0.02, -0.12),
        (0.24, -0.02, -0.18),
        (0.18, 0.0, -0.10),
    ):
        q = arm.ik(*target, seed=ARM_HOME)
        tip, approach = arm.fk(q)
        if np.linalg.norm(tip - np.array(target)) > 1e-4 or approach[2] > -0.98:
            raise AssertionError((target, tip, approach))
    del home_axis


if __name__ == "__main__":
    self_check()
    np.set_printoptions(precision=4, suppress=True)
    print("stance", stance_command())
    print("home tip", ArmKinematics().fk(ARM_HOME)[0])
    print("IK ok")
