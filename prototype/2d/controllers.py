import math
import numpy as np
from sim import DT, MAX_RANGE, MAX_SPEED, MAX_TURN, wrap

def goal_error(state, goal):
    target = math.atan2(goal[1] - state[1], goal[0] - state[0])
    return wrap(target - state[2])

class FlyConnectomeController:
    name = "fly_connectome"
    def reset(self):
        self.prev = None
        self.escape_dir = 1.0

    def act(self, state, ranges, angles, goal):
        if self.prev is None:
            closing = np.zeros_like(ranges)
        else:
            closing = np.maximum(self.prev - ranges, 0.0) / DT
        self.prev = ranges.copy()
        # T4/T5-like local motion -> LPLC2/LC4-like looming pool -> DNp03-like saccade.
        t4t5 = np.clip(closing / (ranges + 0.55), 0.0, 3.0)
        proximity = np.clip((4.0 - ranges) / 4.0, 0.0, 1.0) ** 2
        retinal = t4t5 + 0.28 * proximity
        front_w = np.clip(np.cos(angles), 0.0, 1.0) ** 2
        lplc2 = float(np.tanh(1.8 * np.sum(t4t5 * front_w) / (np.sum(front_w) + 1e-9)))
        lc4 = float(np.tanh(1.35 * np.max(t4t5 * front_w)))
        dnp03 = max(lplc2, lc4)
        asym = float(np.sum(retinal * np.sin(angles)) / (np.sum(retinal) + 1e-9))
        front = ranges[np.abs(angles) < math.radians(22)]
        if float(np.min(front)) < 2.25:
            left = float(np.mean(ranges[angles > 0.18]))
            right = float(np.mean(ranges[angles < -0.18]))
            self.escape_dir = 1.0 if left > right else -1.0
        elif abs(asym) > 0.045:
            self.escape_dir = -np.sign(asym)
        ge = goal_error(state, goal)
        omega = 1.05 * ge - 2.15 * asym
        if dnp03 > 0.42 or float(np.min(front)) < 1.65:
            omega += 1.55 * self.escape_dir
        front_factor = np.clip((float(np.min(front)) - 0.2) / 3.5, 0.32, 1.0)
        speed = MAX_SPEED * front_factor * (1.0 - 0.42 * dnp03)
        return float(np.clip(speed, 0.9, MAX_SPEED)), float(np.clip(omega, -MAX_TURN, MAX_TURN))

class ModifiedFlyController:
    name = "modified_fly"
    defaults = dict(goal_gain=0.95, avoid_gain=3.0, ttc_gain=1.6,
                    prox_gain=0.65, brake_gain=0.72, memory=0.72)
    def __init__(self, params=None):
        self.params = self.defaults | (params or {})
    def reset(self):
        self.prev = None
        self.mem = 0.0
        self.escape_dir = 1.0

    def act(self, state, ranges, angles, goal):
        p = self.params
        if self.prev is None:
            closing = np.zeros_like(ranges)
        else:
            closing = np.maximum(self.prev - ranges, 0.0) / DT
        self.prev = ranges.copy()
        ttc = np.where(closing > 0.03, ranges / (closing + 1e-9), 99.0)
        ttc_risk = np.exp(-ttc / max(p["ttc_gain"], 0.2))
        proximity = np.clip((5.5 - ranges) / 5.5, 0.0, 1.0) ** 2
        retinal = ttc_risk + p["prox_gain"] * proximity
        front_w = np.clip(np.cos(angles), 0.0, 1.0) ** 2
        retinal *= 0.25 + 0.75 * front_w
        denom = np.sum(retinal) + 1e-9
        asym = float(np.sum(retinal * np.sin(angles)) / denom)
        self.mem = p["memory"] * self.mem + (1.0 - p["memory"]) * asym
        front = ranges[np.abs(angles) < math.radians(22)]
        if float(np.min(front)) < 2.6 and abs(self.mem) < 0.04:
            left = float(np.mean(ranges[angles > 0.25]))
            right = float(np.mean(ranges[angles < -0.25]))
            self.escape_dir = 1.0 if left > right else -1.0
            self.mem = -0.18 * self.escape_dir
        elif abs(self.mem) > 0.025:
            self.escape_dir = -np.sign(self.mem)
        ge = goal_error(state, goal)
        threat = float(np.clip(np.max(retinal), 0.0, 1.0))
        away = -p["avoid_gain"] * self.mem
        if threat > 0.72:
            away += 1.3 * self.escape_dir
        omega = p["goal_gain"] * ge + away
        speed = MAX_SPEED * (1.0 - p["brake_gain"] * threat)
        speed *= np.clip(float(np.min(front)) / 3.0, 0.45, 1.0)
        return float(np.clip(speed, 0.9, MAX_SPEED)), float(np.clip(omega, -MAX_TURN, MAX_TURN))

class VFHController:
    name = "classical_vfh"
    def reset(self):
        self.last_angle = 0.0
    def act(self, state, ranges, angles, goal):
        ga = math.atan2(goal[1] - state[1], goal[0] - state[0])
        candidate_global = state[2] + angles
        goal_score = np.cos(np.array([wrap(x - ga) for x in candidate_global]))
        clearance = np.clip(ranges / MAX_RANGE, 0.0, 1.0)
        smooth = np.cos(angles - self.last_angle)
        score = 1.55 * goal_score + 1.55 * clearance + 0.18 * smooth
        score -= np.where(ranges < 2.6, 6.0 * (2.6 - ranges), 0.0)
        idx = int(np.argmax(score))
        chosen = float(angles[idx])
        self.last_angle = chosen
        omega = np.clip(2.6 * chosen, -MAX_TURN, MAX_TURN)
        front = ranges[np.abs(angles) < math.radians(18)]
        speed = MAX_SPEED * np.clip((float(np.min(front)) - 0.25) / 6.0, 0.18, 1.0)
        return float(speed), float(omega)
