from dataclasses import dataclass
import math
import numpy as np

DT = 0.08
DRONE_RADIUS = 0.42
MAX_RANGE = 15.0
MAX_SPEED = 5.0
MAX_ACCEL = 5.0
MAX_YAW_RATE = 2.8
MAX_CLIMB_RATE = 3.0
VEL_TAU = 0.48

AZIMUTHS = np.deg2rad(np.linspace(-80.0, 80.0, 21))
ELEVATIONS = np.deg2rad(np.array([-40.0, -20.0, 0.0, 20.0, 40.0]))
_AZ, _EL = np.meshgrid(AZIMUTHS, ELEVATIONS)
RAY_AZ = _AZ.ravel()
RAY_EL = _EL.ravel()


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class World3D:
    size: np.ndarray
    start: np.ndarray
    goal: np.ndarray
    obstacles: np.ndarray
    difficulty: str
    seed: int

def make_world(seed, difficulty="cluttered"):
    rng = np.random.default_rng(seed)
    counts = {"sparse": 10, "cluttered": 18, "dense": 28}
    size = np.array([70.0, 40.0, 18.0])
    start = np.array([4.0, 20.0, 6.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([64.0, 20.0, 10.0])
    obs = []
    attempts = 0
    while len(obs) < counts[difficulty] and attempts < 6000:
        attempts += 1
        r = float(rng.uniform(1.1, 2.5))
        c = np.array([
            rng.uniform(10.0, 60.0),
            rng.uniform(4.0, 36.0),
            rng.uniform(2.5, 15.5),
        ])
        if np.linalg.norm(c - start[:3]) < r + 5.0:
            continue
        if np.linalg.norm(c - goal) < r + 5.0:
            continue
        if any(np.linalg.norm(c - o[:3]) < r + o[3] + 0.5 for o in obs):
            continue
        obs.append([*c, r])
    return World3D(size, start, goal, np.asarray(obs, dtype=float), difficulty, seed)


def make_demo_world():
    obs = np.array([
        [15, 16, 7, 2.4], [15, 25, 11, 2.2], [26, 20, 7, 2.8],
        [35, 12, 10, 2.3], [35, 27, 6, 2.5], [45, 20, 11, 2.8],
        [54, 14, 7, 2.0], [55, 26, 12, 2.1],
    ], dtype=float)
    size = np.array([70.0, 40.0, 18.0])
    start = np.array([4.0, 20.0, 6.0, 0.0, 0.0, 0.0, 0.0])
    return World3D(size, start, np.array([64.0, 20.0, 10.0]), obs, "demo", 999)

def ray_directions(yaw):
    h = yaw + RAY_AZ
    ce = np.cos(RAY_EL)
    return np.column_stack((ce * np.cos(h), ce * np.sin(h), np.sin(RAY_EL)))


def raycast(state, world):
    p = state[:3]
    dirs = ray_directions(state[6])
    best = np.full(len(dirs), MAX_RANGE, dtype=float)
    if len(world.obstacles):
        oc = p[None, None, :] - world.obstacles[None, :, :3]
        d = dirs[:, None, :]
        b = 2.0 * np.sum(d * oc, axis=2)
        c = np.sum(oc * oc, axis=2) - (world.obstacles[None, :, 3] + DRONE_RADIUS) ** 2
        disc = b * b - 4.0 * c
        roots = np.where(disc >= 0.0, (-b - np.sqrt(np.maximum(disc, 0.0))) / 2.0, np.inf)
        roots = np.where(roots > 0.0, roots, np.inf)
        best = np.minimum(best, np.min(roots, axis=1))
    eps = 1e-9
    lo = np.array([DRONE_RADIUS] * 3)
    hi = world.size - DRONE_RADIUS
    for axis in range(3):
        comp = dirs[:, axis]
        t = np.full(len(dirs), np.inf)
        pos = comp > eps
        neg = comp < -eps
        t[pos] = (hi[axis] - p[axis]) / comp[pos]
        t[neg] = (lo[axis] - p[axis]) / comp[neg]
        best = np.minimum(best, np.where(t > 0.0, t, np.inf))
    return np.clip(best, 0.0, MAX_RANGE), RAY_AZ.copy(), RAY_EL.copy()

def step(state, speed_cmd, yaw_rate_cmd, climb_cmd, world):
    speed_cmd = float(np.clip(speed_cmd, 0.0, MAX_SPEED))
    yaw_rate_cmd = float(np.clip(yaw_rate_cmd, -MAX_YAW_RATE, MAX_YAW_RATE))
    climb_cmd = float(np.clip(climb_cmd, -MAX_CLIMB_RATE, MAX_CLIMB_RATE))
    yaw = wrap(state[6] + yaw_rate_cmd * DT)
    desired_v = np.array([
        speed_cmd * math.cos(yaw),
        speed_cmd * math.sin(yaw),
        climb_cmd,
    ])
    accel = (desired_v - state[3:6]) / VEL_TAU
    a_norm = np.linalg.norm(accel)
    if a_norm > MAX_ACCEL:
        accel *= MAX_ACCEL / a_norm
    vel = state[3:6] + accel * DT
    v_norm = np.linalg.norm(vel)
    if v_norm > MAX_SPEED * 1.15:
        vel *= (MAX_SPEED * 1.15) / v_norm
    pos = state[:3] + vel * DT
    new_state = np.r_[pos, vel, yaw]
    collision = bool(np.any(pos < DRONE_RADIUS) or np.any(pos > world.size - DRONE_RADIUS))
    if len(world.obstacles):
        d = np.linalg.norm(world.obstacles[:, :3] - pos, axis=1)
        collision = collision or bool(np.any(d <= world.obstacles[:, 3] + DRONE_RADIUS))
    success = bool(np.linalg.norm(pos - world.goal) <= 1.5)
    return new_state, collision, success


def goal_angles(state, goal):
    delta = goal - state[:3]
    yaw_target = math.atan2(delta[1], delta[0])
    horiz = math.hypot(delta[0], delta[1])
    elev_target = math.atan2(delta[2], max(horiz, 1e-9))
    return wrap(yaw_target - state[6]), elev_target