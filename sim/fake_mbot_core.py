"""ROS-free core of the fake MBot: world map, ray casting, diff-drive motion.

Kept separate from the ROS node so it can be tested without ROS.
"""

import math
import os

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_pgm(path):
    """Read a binary (P5) or ASCII (P2) PGM image as a uint8 array, row 0 = top."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        # skip whitespace and comments
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b'P5':
        pos += 1  # single whitespace after maxval
        dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
        img = np.frombuffer(data, dtype=dtype, count=w * h, offset=pos).reshape(h, w)
    elif magic == b'P2':
        img = np.array(data[pos:].split()[:w * h], dtype=np.int32).reshape(h, w)
    else:
        raise ValueError(f'{path}: not a PGM file (magic {magic!r})')
    return (img.astype(np.float64) * 255.0 / maxval).astype(np.uint8)


class World:
    """Ground-truth occupancy from a map_server-style YAML + PGM."""

    def __init__(self, yaml_path, unknown_is_wall=True):
        import yaml  # python3-yaml ships with ROS
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        img = read_pgm(img_path)

        negate = int(meta.get('negate', 0))
        p = img.astype(np.float64) / 255.0
        occ_prob = p if negate else 1.0 - p
        occupied = occ_prob > float(meta.get('occupied_thresh', 0.65))
        free = occ_prob < float(meta.get('free_thresh', 0.25))

        # Flip so that row index grows with +y (image row 0 is the top of the map).
        self.occupied = np.flipud(occupied)
        self.free = np.flipud(free)
        self.blocked = self.occupied | (~self.free if unknown_is_wall else False)

        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        if len(meta['origin']) > 2 and abs(float(meta['origin'][2])) > 1e-9:
            raise ValueError('Rotated map origins are not supported')
        self.h, self.w = self.blocked.shape
        self.diag = math.hypot(self.w, self.h) * self.res

    def to_cell(self, x, y):
        gx = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        gy = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        return gx, gy

    def is_blocked(self, x, y):
        """True for wall cells and for anything outside the map."""
        gx, gy = self.to_cell(x, y)
        inb = (gx >= 0) & (gx < self.w) & (gy >= 0) & (gy < self.h)
        out = np.ones(np.shape(gx), dtype=bool)
        out[inb] = self.blocked[gy[inb], gx[inb]]
        return out

    def is_free(self, x, y):
        gx, gy = self.to_cell(x, y)
        inb = (gx >= 0) & (gx < self.w) & (gy >= 0) & (gy < self.h)
        out = np.zeros(np.shape(gx), dtype=bool)
        out[inb] = self.free[gy[inb], gx[inb]]
        return out

    def cast(self, x, y, angles, range_min, range_max):
        """Range to the first wall along each world-frame angle; inf if none."""
        angles = np.asarray(angles, dtype=np.float64)
        max_r = min(range_max, self.diag + 0.1)
        step = self.res * 0.5
        d = np.arange(range_min, max_r + step, step)
        xs = x + np.outer(np.cos(angles), d)
        ys = y + np.outer(np.sin(angles), d)
        gx, gy = self.to_cell(xs, ys)
        inb = (gx >= 0) & (gx < self.w) & (gy >= 0) & (gy < self.h)
        hit = np.zeros(xs.shape, dtype=bool)
        hit[inb] = self.blocked[gy[inb], gx[inb]]
        # Leaving the map counts as "nothing hit" (max range).
        any_hit = hit.any(axis=1)
        first = hit.argmax(axis=1)
        r = np.full(len(angles), np.inf)
        r[any_hit] = d[first[any_hit]]
        r[r > range_max] = np.inf
        return r

    def collides(self, x, y, radius):
        """True if a disc of the given radius at (x, y) touches a wall."""
        pts = [(x, y)]
        for r in (radius * 0.5, radius):
            n = max(8, int(2 * math.pi * r / self.res))
            a = np.linspace(0, 2 * math.pi, n, endpoint=False)
            pts.append((x + r * np.cos(a), y + r * np.sin(a)))
        return any(bool(np.any(self.is_blocked(px, py))) for px, py in pts)


class DiffDrive:
    """True robot pose plus a separately integrated, noisy odometry pose.

    Commands pass through acceleration limits so the motion looks like the real
    robot ramping up instead of teleporting to the commanded speed.
    """

    def __init__(self, x, y, theta, max_v, max_w, max_accel, max_alpha,
                 odom_v_noise, odom_w_noise, rng):
        self.x, self.y, self.theta = x, y, theta          # ground truth (world frame)
        self.ox_, self.oy_, self.oth = 0.0, 0.0, 0.0       # odometry (odom frame, starts at 0)
        self.v = self.w = 0.0                              # actual velocities
        self.cmd_v = self.cmd_w = 0.0
        self.max_v, self.max_w = max_v, max_w
        self.max_accel, self.max_alpha = max_accel, max_alpha
        self.odom_v_noise, self.odom_w_noise = odom_v_noise, odom_w_noise
        self.rng = rng
        # Per-run wheel calibration error, like a slightly wrong wheel radius / base width.
        self.v_scale = 1.0 + rng.normal(0.0, odom_v_noise)
        self.w_scale = 1.0 + rng.normal(0.0, odom_w_noise)
        self.bumped = False
        self.odom_v = self.odom_w = 0.0

    def set_cmd(self, v, w):
        self.cmd_v = float(np.clip(v, -self.max_v, self.max_v))
        self.cmd_w = float(np.clip(w, -self.max_w, self.max_w))

    def step(self, dt, world=None, radius=0.0):
        dv = np.clip(self.cmd_v - self.v, -self.max_accel * dt, self.max_accel * dt)
        dw = np.clip(self.cmd_w - self.w, -self.max_alpha * dt, self.max_alpha * dt)
        self.v += dv
        self.w += dw

        # Ground truth: midpoint integration.
        th_mid = self.theta + 0.5 * self.w * dt
        nx = self.x + self.v * math.cos(th_mid) * dt
        ny = self.y + self.v * math.sin(th_mid) * dt
        nth = wrap(self.theta + self.w * dt)

        self.bumped = False
        if world is not None and radius > 0.0 and world.collides(nx, ny, radius):
            # Blocked by a wall: rotation still happens, translation does not.
            self.bumped = True
            nx, ny = self.x, self.y
            self.v = 0.0
        self.x, self.y, self.theta = nx, ny, nth

        # Odometry: what the encoders think happened (wheels spin even when bumped).
        v_meas = self.v if not self.bumped else self.cmd_v
        v_o = v_meas * self.v_scale + self.rng.normal(0.0, self.odom_v_noise * abs(v_meas))
        w_o = self.w * self.w_scale + self.rng.normal(0.0, self.odom_w_noise * abs(self.w))
        th_mid_o = self.oth + 0.5 * w_o * dt
        self.ox_ += v_o * math.cos(th_mid_o) * dt
        self.oy_ += v_o * math.sin(th_mid_o) * dt
        self.oth = wrap(self.oth + w_o * dt)
        self.odom_v, self.odom_w = v_o, w_o
