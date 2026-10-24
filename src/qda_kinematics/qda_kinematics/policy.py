"""Small residual network used as the learned locomotion layer.

The gait in ``kinematics.joint_command`` is the base policy. This network adds a
bounded joint residual from body tilt, which is the same role a trained
quadruped policy plays at deployment: observation in, joint targets out.
Weights are produced by ``python3 -m qda_kinematics.policy`` (imitation of a
tilt stabilizer) and stored next to this file.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

OBS_DIM = 32
ACT_DIM = 12
HIDDEN = 64

# Observation layout:
# 0:3   gravity in the base frame
# 3:6   commanded vx, vy, yaw rate
# 6:18  leg joint positions
# 18:30 leg joint velocities
# 30:32 trot phase sin, cos


def default_path() -> Path:
    return Path(__file__).with_name("residual_policy.npz")


def stabilizer_target(gravity_xy: np.ndarray) -> np.ndarray:
    """Thigh/hip corrections that shift the feet against a body tilt."""
    residual = np.zeros(ACT_DIM, dtype=np.float64)
    gx, gy = float(gravity_xy[0]), float(gravity_xy[1])
    for leg in range(4):
        residual[3 * leg + 0] = 0.28 * gy
        residual[3 * leg + 1] = 0.42 * gx
        residual[3 * leg + 2] = -0.20 * gx
    return residual


class ResidualPolicy:
    def __init__(self, path: Path | None = None):
        file_path = Path(path) if path is not None else default_path()
        if not file_path.is_file():
            raise FileNotFoundError(file_path)
        data = np.load(file_path)
        self.w1 = data["w1"]
        self.b1 = data["b1"]
        self.w2 = data["w2"]
        self.b2 = data["b2"]
        self.w3 = data["w3"]
        self.b3 = data["b3"]
        self.limit = float(data["limit"]) if "limit" in data.files else 0.18

    def act(self, observation: np.ndarray) -> np.ndarray:
        x = np.asarray(observation, dtype=np.float64).reshape(-1)
        if x.shape[0] != OBS_DIM:
            raise ValueError(f"expected {OBS_DIM} observations, got {x.shape[0]}")
        h1 = np.tanh(self.w1 @ x + self.b1)
        h2 = np.tanh(self.w2 @ h1 + self.b2)
        residual = self.w3 @ h2 + self.b3
        return np.clip(residual, -self.limit, self.limit)


def _train(path: Path, seed: int = 7) -> None:
    rng = np.random.default_rng(seed)
    w1 = rng.normal(0.0, 0.15, size=(HIDDEN, OBS_DIM))
    b1 = np.zeros(HIDDEN)
    w2 = rng.normal(0.0, 0.15, size=(HIDDEN, HIDDEN))
    b2 = np.zeros(HIDDEN)
    w3 = rng.normal(0.0, 0.05, size=(ACT_DIM, HIDDEN))
    b3 = np.zeros(ACT_DIM)

    def forward(x: np.ndarray):
        z1 = w1 @ x + b1
        h1 = np.tanh(z1)
        z2 = w2 @ h1 + b2
        h2 = np.tanh(z2)
        y = w3 @ h2 + b3
        return y, h1, h2, z1, z2

    lr = 0.03
    for step in range(2500):
        gravity = rng.normal(0.0, 0.18, size=2)
        gravity = np.clip(gravity, -0.45, 0.45)
        obs = np.zeros(OBS_DIM)
        obs[0:2] = gravity
        obs[2] = -1.0 + rng.normal(0.0, 0.02)
        obs[3:6] = rng.normal(0.0, 0.08, size=3)
        obs[6:18] = rng.normal(0.0, 0.15, size=12)
        obs[18:30] = rng.normal(0.0, 0.4, size=12)
        obs[30:32] = rng.normal(0.0, 0.7, size=2)
        target = stabilizer_target(gravity)
        pred, h1, h2, z1, z2 = forward(obs)
        err = pred - target
        # Mean-square loss, batch size 1.
        dy = (2.0 / ACT_DIM) * err
        dw3 = np.outer(dy, h2)
        db3 = dy
        dh2 = w3.T @ dy
        dz2 = dh2 * (1.0 - np.tanh(z2) ** 2)
        dw2 = np.outer(dz2, h1)
        db2 = dz2
        dh1 = w2.T @ dz2
        dz1 = dh1 * (1.0 - np.tanh(z1) ** 2)
        dw1 = np.outer(dz1, obs)
        db1 = dz1
        w3 -= lr * dw3
        b3 -= lr * db3
        w2 -= lr * dw2
        b2 -= lr * db2
        w1 -= lr * dw1
        b1 -= lr * db1
        if step in (0, 400, 1200, 2499):
            loss = float(np.mean(err ** 2))
            print(f"step {step} loss {loss:.6f}")

    # Hold-out error should stay well under a tenth of a radian.
    errors = []
    for _ in range(200):
        gravity = rng.normal(0.0, 0.15, size=2)
        obs = np.zeros(OBS_DIM)
        obs[0:2] = gravity
        obs[2] = -1.0
        pred, *_ = forward(obs)
        errors.append(np.mean((pred - stabilizer_target(gravity)) ** 2))
    print(f"holdout mse {float(np.mean(errors)):.6f}")
    np.savez(
        path,
        w1=w1,
        b1=b1,
        w2=w2,
        b2=b2,
        w3=w3,
        b3=b3,
        limit=np.array(0.18),
    )
    print(f"wrote {path}")


if __name__ == "__main__":
    _train(default_path())
