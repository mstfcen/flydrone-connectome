from dataclasses import dataclass
import math
import numpy as np

DT = 0.08
DRONE_RADIUS = 0.45
MAX_SPEED = 4.0
MAX_TURN = 3.8
RAY_ANGLES = np.linspace(-math.pi / 2, math.pi / 2, 31)
MAX_RANGE = 12.0

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

@dataclass
class World:
    width: float
    height: float
    start: np.ndarray
    goal: np.ndarray
    obstacles: np.ndarray
    difficulty: str
    seed: int

def make_world(seed, difficulty="cluttered"):
    rng = np.random.default_rng(seed)
    counts = {"sparse": 8, "cluttered": 14, "dense": 20}
    w, h = 60.0, 36.0
    start = np.array([4.0, h / 2, 0.0], dtype=float)
    goal = np.array([w - 6.0, h / 2], dtype=float)
    obs = []
    target = counts[difficulty]
    attempts = 0
    while len(obs) < target and attempts < 3000:
        attempts += 1
        r = rng.uniform(1.0, 2.2)
        x = rng.uniform(9.0, w - 9.0)
        y = rng.uniform(3.0, h - 3.0)
        c = np.array([x, y])
        if np.linalg.norm(c - start[:2]) < r + 5.0:
            continue
        if np.linalg.norm(c - goal) < r + 5.0:
            continue
        if any(np.linalg.norm(c - o[:2]) < r + o[2] + 0.7 for o in obs):
            continue
        obs.append([x, y, r])
    return World(w, h, start, goal, np.asarray(obs, dtype=float), difficulty, seed)

def make_demo_world():
    obs = np.array([
        [14, 14, 2.2], [14, 23, 2.0], [23, 18, 2.4],
        [32, 11, 2.1], [32, 25, 2.3], [41, 17, 2.5],
        [49, 12, 1.8], [49, 24, 1.9],
    ], dtype=float)
    return World(60.0, 36.0, np.array([4.0, 18.0, 0.0]),
                 np.array([54.0, 18.0]), obs, "demo", 999)

def raycast(state, world):
    p = state[:2]
    headings = state[2] + RAY_ANGLES
    dirs = np.column_stack((np.cos(headings), np.sin(headings)))
    best = np.full(len(RAY_ANGLES), MAX_RANGE, dtype=float)
    if len(world.obstacles):
        oc = p[None, None, :] - world.obstacles[None, :, :2]
        d = dirs[:, None, :]
        b = 2.0 * np.sum(d * oc, axis=2)
        c = np.sum(oc * oc, axis=2) - (world.obstacles[None, :, 2] + DRONE_RADIUS) ** 2
        disc = b * b - 4.0 * c
        valid = disc >= 0
        roots = np.where(valid, (-b - np.sqrt(np.maximum(disc, 0.0))) / 2.0, np.inf)
        roots = np.where(roots > 0, roots, np.inf)
        best = np.minimum(best, np.min(roots, axis=1))
    eps = 1e-9
    dx, dy = dirs[:, 0], dirs[:, 1]
    def wall_distance(numerator, denominator, mask):
        out = np.full_like(denominator, np.inf, dtype=float)
        np.divide(numerator, denominator, out=out, where=mask)
        return out

    wall_ts = []
    wall_ts.append(wall_distance(world.width - DRONE_RADIUS - p[0], dx, dx > eps))
    wall_ts.append(wall_distance(DRONE_RADIUS - p[0], dx, dx < -eps))
    wall_ts.append(wall_distance(world.height - DRONE_RADIUS - p[1], dy, dy > eps))
    wall_ts.append(wall_distance(DRONE_RADIUS - p[1], dy, dy < -eps))
    best = np.minimum(best, np.min(np.vstack(wall_ts), axis=0))
    return np.clip(best, 0.0, MAX_RANGE), RAY_ANGLES.copy()

def step(state, speed, omega, world):
    speed = float(np.clip(speed, 0.0, MAX_SPEED))
    omega = float(np.clip(omega, -MAX_TURN, MAX_TURN))
    heading = wrap(state[2] + omega * DT)
    pos = state[:2] + speed * DT * np.array([math.cos(heading), math.sin(heading)])
    new_state = np.array([pos[0], pos[1], heading], dtype=float)
    collision = (
        pos[0] < DRONE_RADIUS or pos[0] > world.width - DRONE_RADIUS
        or pos[1] < DRONE_RADIUS or pos[1] > world.height - DRONE_RADIUS
    )
    if len(world.obstacles):
        d = np.linalg.norm(world.obstacles[:, :2] - pos, axis=1)
        collision = collision or bool(np.any(d <= world.obstacles[:, 2] + DRONE_RADIUS))
    success = np.linalg.norm(pos - world.goal) <= 1.2
    return new_state, collision, success
