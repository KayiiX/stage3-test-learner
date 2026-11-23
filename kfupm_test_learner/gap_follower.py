"""Reactive driving from LiDAR only: follow-the-gap.

1. Look at the beams within +-100 degrees of the nose, limited to 10 m.
2. Put a "bubble" around the closest obstacle: beams pointing at it count as blocked,
   so the car never aims next to a wall it is about to touch.
3. Smooth the remaining distances and steer towards the deepest free direction.
4. Drive faster when the way ahead is long and the wheel is straight.
"""
import math

import numpy as np

FOV = math.radians(100)    # look +-100 degrees around the nose
BUBBLE = 0.40              # m of clearance around the closest obstacle
MAX_STEER = 0.41           # rad (simulator limit)


def gap_command(scan, v_min=1.5, v_max=5.0):
    r = np.nan_to_num(np.asarray(scan.ranges, dtype=float), nan=0.0, posinf=scan.range_max)
    ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    keep = np.abs(ang) <= FOV
    r, ang = np.clip(r[keep], 0.0, 10.0), ang[keep]
    free = r.copy()
    i0 = int(np.argmin(free))
    half = int(BUBBLE / max(free[i0], 0.1) / scan.angle_increment) + 1
    free[max(0, i0 - half):i0 + half] = 0.0
    k = 15
    smooth = np.convolve(free, np.ones(k) / k, mode="same")
    steer = float(np.clip(ang[int(np.argmax(smooth))] * 0.8, -MAX_STEER, MAX_STEER))
    front = np.abs(ang) < math.radians(10)
    ahead = float(np.min(r[front])) if np.any(front) else 1.0
    speed = float(np.clip(0.8 * ahead, v_min, v_max)) * (1.0 - 0.45 * abs(steer) / MAX_STEER)
    return steer, speed, ahead
