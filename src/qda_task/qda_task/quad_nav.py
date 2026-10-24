"""Quadruped base planner: A* on the occupancy grid plus a strafing local planner.

The body can move forward and sideways. A sampled step is kept only when the
whole footprint stays out of inflated obstacles.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

# Nose is longer than the hips. Values are meters in the base frame.
FOOTPRINT = ((0.42, 0.20), (0.42, -0.20), (-0.34, -0.20), (-0.34, 0.20))
INFLATE_M = 0.34


def plan_path(grid: np.ndarray, origin, resolution: float, start, goal, inflate_m: float = INFLATE_M):
    """Return a sparse polyline from start to goal. Unknown cells are free."""
    goal = np.asarray(goal, dtype=float)
    start = np.asarray(start, dtype=float)
    if grid is None or grid.size == 0:
        return [goal]
    height, width = grid.shape
    occupied = grid >= 50
    radius = max(1, int(round(inflate_m / resolution)))
    inflated = _inflate(occupied, radius)
    ox, oy = origin

    def cell(xy):
        return int((xy[0] - ox) / resolution), int((xy[1] - oy) / resolution)

    def world(col, row):
        return np.array([ox + (col + 0.5) * resolution, oy + (row + 0.5) * resolution])

    sc, sr = cell(start)
    gc, gr = cell(goal)
    if not (0 <= sc < width and 0 <= sr < height and 0 <= gc < width and 0 <= gr < height):
        return [goal]
    gc, gr = _nearest_free(inflated, gc, gr, 12)
    inflated[sr, sc] = False
    inflated[gr, gc] = False
    start_key = (sc, sr)
    goal_key = (gc, gr)
    open_set = [(0.0, start_key)]
    came = {}
    cost = {start_key: 0.0}
    found = start_key == goal_key
    while open_set and len(cost) < 60000:
        _, current = heapq.heappop(open_set)
        if current == goal_key:
            found = True
            break
        cx, cy = current
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height or inflated[ny, nx]:
                continue
            step = 1.414 if dx and dy else 1.0
            nxt = cost[current] + step
            key = (nx, ny)
            if nxt >= cost.get(key, 1e9):
                continue
            cost[key] = nxt
            came[key] = current
            heapq.heappush(open_set, (nxt + math.hypot(nx - gc, ny - gr), key))
    if not found:
        return None
    chain = [goal_key]
    while chain[-1] != start_key:
        chain.append(came[chain[-1]])
    chain.reverse()
    sparse = [world(col, row) for col, row in chain[::4]]
    sparse.append(world(gc, gr))
    return sparse


def steer(pos, yaw: float, path, grid, origin, resolution: float) -> tuple[float, float, float]:
    """Pick a body-frame vx, vy, wz that follows the path and clears the body."""
    pos = np.asarray(pos, dtype=float)
    if not path:
        return 0.0, 0.0, 0.0
    aim = _lookahead(pos, path, 0.55)
    heading = math.atan2(aim[1] - pos[1], aim[0] - pos[0])
    steer_err = _wrap(heading - yaw)
    dist = float(np.linalg.norm(np.asarray(path[-1]) - pos))
    samples = []
    for vx in (0.0, 0.08, 0.15):
        for vy in (-0.10, 0.0, 0.10):
            for wz in (-0.40, 0.0, 0.40):
                if vx == 0.0 and vy == 0.0 and wz == 0.0:
                    continue
                if abs(steer_err) > 0.8 and vx > 0.08:
                    continue
                samples.append((vx, vy, wz))
    # Bias the search toward the path heading and a sideways dodge.
    samples.append((0.12, 0.0, float(np.clip(1.1 * steer_err, -0.45, 0.45))))
    best = None
    best_score = -1e9
    for vx, vy, wz in samples:
        if _hits(pos, yaw, vx, vy, wz, grid, origin, resolution):
            continue
        end = _roll(pos, yaw, vx, vy, wz, 0.8)
        progress = float(np.linalg.norm(aim - pos) - np.linalg.norm(aim - end))
        score = 2.2 * progress - 0.15 * abs(vy) - 0.05 * abs(wz) - 0.2 * abs(_wrap(math.atan2(aim[1] - end[1], aim[0] - end[0]) - (yaw + wz * 0.8)))
        if dist < 0.35:
            score += 0.4 * vx
        if score > best_score:
            best_score = score
            best = (vx, vy, wz)
    if best is not None:
        return best
    # Nothing is clear ahead. Turn toward the side with more free space.
    left = _free_side(pos, yaw, grid, origin, resolution, 1.0)
    right = _free_side(pos, yaw, grid, origin, resolution, -1.0)
    return 0.0, 0.0, 0.45 if left >= right else -0.45


def _lookahead(pos, path, distance: float):
    nearest = min(range(len(path)), key=lambda i: float(np.linalg.norm(path[i] - pos)))
    for point in path[nearest:]:
        if float(np.linalg.norm(point - pos)) >= distance:
            return point
    return path[-1]


def _hits(pos, yaw, vx, vy, wz, grid, origin, resolution) -> bool:
    if grid is None:
        return False
    for horizon in (0.0, 0.25, 0.55, 0.9):
        pose = _roll(pos, yaw, vx, vy, wz, horizon)
        pose_yaw = yaw + wz * horizon
        if not _footprint_free(pose, pose_yaw, grid, origin, resolution):
            return True
    return False


def _footprint_free(pos, yaw, grid, origin, resolution) -> bool:
    c, s = math.cos(yaw), math.sin(yaw)
    height, width = grid.shape
    ox, oy = origin
    for lx, ly in FOOTPRINT:
        x = pos[0] + c * lx - s * ly
        y = pos[1] + s * lx + c * ly
        col = int((x - ox) / resolution)
        row = int((y - oy) / resolution)
        if row < 0 or col < 0 or row >= height or col >= width:
            continue
        if grid[row, col] >= 50:
            return False
    return True


def _free_side(pos, yaw, grid, origin, resolution, sign: float) -> int:
    if grid is None:
        return 0
    c, s = math.cos(yaw), math.sin(yaw)
    free = 0
    for dist in (0.4, 0.7, 1.0):
        x = pos[0] + c * 0.3 - s * sign * dist
        y = pos[1] + s * 0.3 + c * sign * dist
        col = int((x - origin[0]) / resolution)
        row = int((y - origin[1]) / resolution)
        if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] and grid[row, col] < 50:
            free += 1
    return free


def _roll(pos, yaw, vx, vy, wz, horizon):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([pos[0] + (c * vx - s * vy) * horizon, pos[1] + (s * vx + c * vy) * horizon])


def _inflate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask.copy()
    out = mask.copy()
    ys, xs = np.where(mask)
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            yy = ys + dy
            xx = xs + dx
            ok = (yy >= 0) & (yy < height) & (xx >= 0) & (xx < width)
            out[yy[ok], xx[ok]] = True
    return out


def _nearest_free(inflated: np.ndarray, col: int, row: int, limit: int):
    if not inflated[row, col]:
        return col, row
    height, width = inflated.shape
    for radius in range(1, limit + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                rr, cc = row + dy, col + dx
                if 0 <= rr < height and 0 <= cc < width and not inflated[rr, cc]:
                    return cc, rr
    return col, row


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
