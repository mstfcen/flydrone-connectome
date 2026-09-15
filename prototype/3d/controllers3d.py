import json
import math
from pathlib import Path
import numpy as np

from sim3d import (
    DT, MAX_CLIMB_RATE, MAX_RANGE, MAX_SPEED, MAX_YAW_RATE,
    RAY_AZ, RAY_EL, goal_angles,
)


def _closing(prev, ranges):
    if prev is None:
        return np.zeros_like(ranges)
    return np.maximum(prev - ranges, 0.0) / DT


def _front_mask(az, el):
    return (np.abs(az) < math.radians(24)) & (np.abs(el) < math.radians(22))


def _free_axes(ranges, az, el):
    left = np.mean(ranges[az > math.radians(18)])
    right = np.mean(ranges[az < -math.radians(18)])
    up = np.mean(ranges[el > math.radians(12)])
    down = np.mean(ranges[el < -math.radians(12)])
    return (1.0 if left > right else -1.0), (1.0 if up > down else -1.0)


class Fly3DController:
    name = "fly_heuristic_3d"

    def reset(self):
        self.prev = None
        self.escape_yaw = 1.0
        self.escape_z = 1.0
    def act(self, state, ranges, az, el, goal):
        closing = _closing(self.prev, ranges)
        self.prev = ranges.copy()
        looming = np.clip(closing / (ranges + 0.55), 0.0, 4.0)
        proximity = np.clip((4.5 - ranges) / 4.5, 0.0, 1.0) ** 2
        retinal = looming + 0.28 * proximity
        denom = np.sum(retinal) + 1e-9
        yaw_asym = float(np.sum(retinal * np.sin(az) * np.cos(el)) / denom)
        z_asym = float(np.sum(retinal * np.sin(el)) / denom)
        front = _front_mask(az, el)
        front_min = float(np.min(ranges[front]))
        if front_min < 2.6:
            self.escape_yaw, self.escape_z = _free_axes(ranges, az, el)
        gy, ge = goal_angles(state, goal)
        threat = float(np.tanh(1.5 * np.max(looming[front])))
        yaw_rate = 1.15 * gy - 2.4 * yaw_asym
        climb = 2.0 * ge - 2.2 * z_asym
        if threat > 0.45 or front_min < 1.8:
            yaw_rate += 1.5 * self.escape_yaw
            climb += 0.9 * self.escape_z
        speed = MAX_SPEED * np.clip(front_min / 4.2, 0.28, 1.0) * (1.0 - 0.48 * threat)
        return (
            float(np.clip(speed, 0.8, MAX_SPEED)),
            float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)),
            float(np.clip(climb, -MAX_CLIMB_RATE, MAX_CLIMB_RATE)),
        )


class ModifiedFly3DController:
    name = "modified_fly_3d"

    def reset(self):
        self.prev = None
        self.mem_yaw = 0.0
        self.mem_z = 0.0
        self.escape_yaw = 1.0
        self.escape_z = 1.0
    def act(self, state, ranges, az, el, goal):
        closing = _closing(self.prev, ranges)
        self.prev = ranges.copy()
        ttc = np.where(closing > 0.03, ranges / (closing + 1e-9), 99.0)
        ttc_risk = np.exp(-ttc / 1.45)
        proximity = np.clip((6.0 - ranges) / 6.0, 0.0, 1.0) ** 2
        retinal = ttc_risk + 0.62 * proximity
        forward = np.clip(np.cos(az) * np.cos(el), 0.0, 1.0)
        retinal *= 0.22 + 0.78 * forward ** 2
        denom = np.sum(retinal) + 1e-9
        yaw_asym = float(np.sum(retinal * np.sin(az)) / denom)
        z_asym = float(np.sum(retinal * np.sin(el)) / denom)
        self.mem_yaw = 0.76 * self.mem_yaw + 0.24 * yaw_asym
        self.mem_z = 0.72 * self.mem_z + 0.28 * z_asym
        front = _front_mask(az, el)
        front_min = float(np.min(ranges[front]))
        if front_min < 2.9:
            self.escape_yaw, self.escape_z = _free_axes(ranges, az, el)
        gy, ge = goal_angles(state, goal)
        threat = float(np.clip(np.max(retinal[front]), 0.0, 1.0))
        yaw_rate = 1.08 * gy - 3.0 * self.mem_yaw
        climb = 2.5 * ge - 2.8 * self.mem_z
        if threat > 0.68:
            yaw_rate += 1.25 * self.escape_yaw
            climb += 0.75 * self.escape_z
        speed = MAX_SPEED * (1.0 - 0.68 * threat)
        speed *= np.clip(front_min / 3.5, 0.34, 1.0)
        return (
            float(np.clip(speed, 0.8, MAX_SPEED)),
            float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)),
            float(np.clip(climb, -MAX_CLIMB_RATE, MAX_CLIMB_RATE)),
        )

class FlyWireGraph3DController:
    name = "flywire_graph_3d"

    def __init__(self, circuit_path=None):
        if circuit_path is None:
            circuit_path = Path(__file__).resolve().parents[2] / "out" / "fafb_loom_dnp03_circuit.json"
        data = json.loads(Path(circuit_path).read_text(encoding="utf-8"))
        self.nodes = data["nodes"]
        self.index = {n["id"]: i for i, n in enumerate(self.nodes)}
        self.visual = np.array([
            i for i, n in enumerate(self.nodes) if n["role"] in {"LC4", "LPLC2"}
        ], dtype=int)
        pre, post, weights = [], [], []
        for e in data["edges"]:
            if e["pre"] in self.index and e["post"] in self.index:
                pre.append(self.index[e["pre"]])
                post.append(self.index[e["post"]])
                weights.append(float(e["weight"]) * float(e["sign"]))
        self.pre = np.asarray(pre, dtype=int)
        self.post = np.asarray(post, dtype=int)
        self.weights = np.asarray(weights, dtype=float)
        dnp = [(i, n["side"]) for i, n in enumerate(self.nodes) if n["role"] == "DNp03"]
        self.dnp_left = next(i for i, s in dnp if s == "left")
        self.dnp_right = next(i for i, s in dnp if s == "right")
        self.kernel = self._make_retinotopic_kernel()

    def _make_retinotopic_kernel(self):
        rows = []
        for i in self.visual:
            n = self.nodes[i]
            raw = int(n["id"][-9:])
            center = 0.48 if n["side"] == "left" else -0.48
            jitter = ((raw % 1009) / 1008.0 - 0.5) * 1.10
            pref_az = float(np.clip(center + jitter, -1.30, 1.30))
            pref_el = float(np.deg2rad([-40, -20, 0, 20, 40][(raw // 1009) % 5]))
            k = np.exp(-0.5 * ((RAY_AZ - pref_az) / 0.36) ** 2
                       -0.5 * ((RAY_EL - pref_el) / 0.31) ** 2)
            rows.append(k / (np.sum(k) + 1e-9))
        return np.asarray(rows)
    def reset(self):
        self.prev = None
        self.escape_yaw = 1.0
        self.last_dnp = (0.0, 0.0)

    def _network(self, retinal):
        visual_drive = self.kernel @ retinal
        a = np.zeros(len(self.nodes), dtype=float)
        a[self.visual] = visual_drive
        for _ in range(4):
            incoming = np.zeros_like(a)
            np.add.at(incoming, self.post, a[self.pre] * self.weights)
            nxt = np.tanh(1.8 * incoming)
            nxt[self.visual] = np.maximum(nxt[self.visual], visual_drive)
            a = 0.30 * a + 0.70 * nxt
        return a

    def act(self, state, ranges, az, el, goal):
        closing = _closing(self.prev, ranges)
        self.prev = ranges.copy()
        ttc = np.where(closing > 0.03, ranges / (closing + 1e-9), 99.0)
        retinal = np.exp(-ttc / 1.45) + 0.50 * np.clip((5.5-ranges)/5.5, 0, 1) ** 2
        retinal *= 0.20 + 0.80 * np.clip(np.cos(az) * np.cos(el), 0, 1) ** 2
        a = self._network(retinal)
        left = float(max(a[self.dnp_left], 0.0))
        right = float(max(a[self.dnp_right], 0.0))
        self.last_dnp = (left, right)
        side_drive = float(np.tanh(4.0 * (left - right)))
        network_threat = float(np.clip(max(left, right) * 2.2, 0.0, 1.0))
        denom = np.sum(retinal) + 1e-9
        z_asym = float(np.sum(retinal * np.sin(el)) / denom)
        front = _front_mask(az, el)
        front_min = float(np.min(ranges[front]))
        if front_min < 2.6:
            self.escape_yaw, _ = _free_axes(ranges, az, el)
        gy, ge = goal_angles(state, goal)
        yaw_rate = 1.05 * gy - 2.7 * side_drive
        if network_threat > 0.58 and abs(side_drive) < 0.05:
            yaw_rate += 1.15 * self.escape_yaw
        climb = 2.4 * ge - 2.5 * z_asym
        threat = max(network_threat, float(np.max(retinal[front])))
        speed = MAX_SPEED * (1.0 - 0.62 * min(threat, 1.0))
        speed *= np.clip(front_min / 3.6, 0.32, 1.0)
        return float(max(speed, 0.8)), float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)), float(np.clip(climb, -MAX_CLIMB_RATE, MAX_CLIMB_RATE))

class VFH3DController:
    name = "classical_vfh_3d"

    def reset(self):
        self.last_az = 0.0
        self.last_el = 0.0

    def act(self, state, ranges, az, el, goal):
        gy, ge = goal_angles(state, goal)
        goal_score = np.cos(az - gy) * np.cos(el - ge)
        clearance = np.clip(ranges / MAX_RANGE, 0.0, 1.0)
        smooth = np.cos(az - self.last_az) * np.cos(el - self.last_el)
        score = 1.65 * goal_score + 1.55 * clearance + 0.20 * smooth
        score -= np.where(ranges < 3.0, 5.5 * (3.0 - ranges), 0.0)
        idx = int(np.argmax(score))
        chosen_az = float(az[idx])
        chosen_el = float(el[idx])
        self.last_az, self.last_el = chosen_az, chosen_el
        yaw_rate = 2.45 * chosen_az
        climb = 3.4 * math.sin(chosen_el)
        front = _front_mask(az, el)
        front_min = float(np.min(ranges[front]))
        speed = MAX_SPEED * np.clip((front_min - 0.25) / 6.0, 0.22, 1.0)
        return (
            float(speed),
            float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)),
            float(np.clip(climb, -MAX_CLIMB_RATE, MAX_CLIMB_RATE)),
        )

class BilateralFlyWire3DController(FlyWireGraph3DController):
    """Engineering variant: mirror the better-connected left DNp03 circuit.

    The FAFB v783 raw subgraph is strongly asymmetric. This controller preserves
    the extracted synaptic topology but evaluates it twice: original retinal
    input for one hemisphere and azimuth-mirrored retinal input for the other.
    It is therefore NOT a raw-connectome result.
    """
    name = "flywire_bilateral_3d"

    @staticmethod
    def _mirror_retina(retinal):
        return retinal.reshape(5, 21)[:, ::-1].ravel()

    def act(self, state, ranges, az, el, goal):
        closing = _closing(self.prev, ranges)
        self.prev = ranges.copy()
        ttc = np.where(closing > 0.03, ranges / (closing + 1e-9), 99.0)
        retinal = np.exp(-ttc / 1.45) + 0.50 * np.clip((5.5-ranges)/5.5, 0, 1) ** 2
        retinal *= 0.20 + 0.80 * np.clip(np.cos(az) * np.cos(el), 0, 1) ** 2
        left = float(max(self._network(retinal)[self.dnp_left], 0.0))
        right = float(max(self._network(self._mirror_retina(retinal))[self.dnp_left], 0.0))
        self.last_dnp = (left, right)
        side_drive = float(np.tanh(4.0 * (left - right)))
        network_threat = float(np.clip(max(left, right) * 2.2, 0.0, 1.0))
        denom = np.sum(retinal) + 1e-9
        z_asym = float(np.sum(retinal * np.sin(el)) / denom)
        front = _front_mask(az, el)
        front_min = float(np.min(ranges[front]))
        if front_min < 2.6:
            self.escape_yaw, _ = _free_axes(ranges, az, el)
        gy, ge = goal_angles(state, goal)
        yaw_rate = 1.05 * gy - 2.7 * side_drive
        if network_threat > 0.58 and abs(side_drive) < 0.05:
            yaw_rate += 1.15 * self.escape_yaw
        climb = 2.4 * ge - 2.5 * z_asym
        threat = max(network_threat, float(np.max(retinal[front])))
        speed = MAX_SPEED * (1.0 - 0.62 * min(threat, 1.0))
        speed *= np.clip(front_min / 3.6, 0.32, 1.0)
        return (
            float(max(speed, 0.8)),
            float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)),
            float(np.clip(climb, -MAX_CLIMB_RATE, MAX_CLIMB_RATE)),
        )