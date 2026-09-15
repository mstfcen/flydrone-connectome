#!/usr/bin/env python3
import argparse
import asyncio
import json
import math
import random
import subprocess
import threading
import time
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

TOPIC = "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
MAX_RANGE = 30.0
GOAL_NORTH_M = 14.0


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


def mean(values):
    return sum(values) / max(len(values), 1)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))
class LidarReader:
    def __init__(self, container):
        self.container = container
        self.lock = threading.Lock()
        self.latest = None
        self.proc = None
        self.thread = None

    def start(self):
        cmd = ["docker", "exec", self.container, "gz", "topic", "-e", "-t", TOPIC]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1)
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        ranges = []
        angle_min = -2.356195
        angle_step = 0.0043673679332715482
        for raw in self.proc.stdout:
            line = raw.strip()
            if line.startswith("angle_min:"):
                angle_min = float(line.split(":", 1)[1])
            elif line.startswith("angle_step:"):
                angle_step = float(line.split(":", 1)[1])
            elif line.startswith("ranges:"):
                token = line.split(":", 1)[1].strip()
                value = MAX_RANGE if token == "inf" else float(token)
                ranges.append(value)
                if len(ranges) == 1080:
                    angles = [angle_min + i * angle_step for i in range(1080)]
                    with self.lock:
                        self.latest = (time.monotonic(), ranges[:], angles)
                    ranges.clear()
    def get(self, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.latest is not None:
                    return self.latest
            time.sleep(0.05)
        raise TimeoutError("No Gazebo lidar scan received")

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class FlightState:
    def __init__(self):
        self.north = 0.0
        self.east = 0.0
        self.down = 0.0
        self.yaw_deg = 0.0
        self.vn = 0.0
        self.ve = 0.0


async def track_position(drone, state):
    async for pv in drone.telemetry.position_velocity_ned():
        state.north = float(pv.position.north_m)
        state.east = float(pv.position.east_m)
        state.down = float(pv.position.down_m)
        state.vn = float(pv.velocity.north_m_s)
        state.ve = float(pv.velocity.east_m_s)


async def track_attitude(drone, state):
    async for att in drone.telemetry.attitude_euler():
        state.yaw_deg = float(att.yaw_deg)
class BaseController:
    name = "base"
    def reset(self):
        pass

    @staticmethod
    def goal_yaw(state):
        dn = GOAL_NORTH_M - state.north
        de = -state.east
        return math.degrees(math.atan2(de, dn))


class NoAvoidController(BaseController):
    name = "no_avoidance"
    def act(self, state, ranges, angles, dt):
        yaw_err = wrap_deg(self.goal_yaw(state) - state.yaw_deg)
        return 2.0, clamp(0.9 * yaw_err, -35.0, 35.0)


class ModifiedFlyController(BaseController):
    name = "modified_fly"
    def reset(self):
        self.prev = None
        self.mem = 0.0
        self.escape_sign = 1.0
        self.escape_until = 0.0

    def act(self, state, ranges, angles, dt):
        if self.prev is None:
            closing = [0.0] * len(ranges)
        else:
            closing = [max(p - r, 0.0) / max(dt, 0.05)
                       for p, r in zip(self.prev, ranges)]
        self.prev = ranges[:]
        risk = []
        for r, c, a in zip(ranges, closing, angles):
            ttc = r / (c + 1e-6) if c > 0.05 else 99.0
            ttc_risk = math.exp(-ttc / 1.5)
            prox = max(0.0, (5.5 - r) / 5.5) ** 2
            front_w = max(0.0, math.cos(a)) ** 2
            risk.append((ttc_risk + 0.65 * prox) * (0.2 + 0.8 * front_w))

        total = sum(risk) + 1e-9
        asym = sum(v * math.sin(a) for v, a in zip(risk, angles)) / total
        self.mem = 0.72 * self.mem + 0.28 * asym
        front = [r for r, a in zip(ranges, angles) if abs(a) < math.radians(24)]
        left = [r for r, a in zip(ranges, angles) if math.radians(25) < a < math.radians(105)]
        right = [r for r, a in zip(ranges, angles) if -math.radians(105) < a < -math.radians(25)]
        front_min = min(front)
        threat = max(risk)
        now = time.monotonic()

        if front_min < 4.0 or threat > 0.55:
            if now >= self.escape_until:
                self.escape_sign = -1.0 if mean(left) > mean(right) else 1.0
            self.escape_until = now + 1.3

        yaw_err = wrap_deg(self.goal_yaw(state) - state.yaw_deg)
        yaw_rate = 0.65 * yaw_err + 85.0 * self.mem
        if now < self.escape_until:
            yaw_rate += 42.0 * self.escape_sign
        speed = 2.2 * (1.0 - 0.62 * clamp(threat, 0.0, 1.0))
        speed *= clamp(front_min / 3.2, 0.28, 1.0)
        return clamp(speed, 0.65, 2.2), clamp(yaw_rate, -60.0, 60.0)
class VFHController(BaseController):
    name = "classical_vfh"
    def reset(self):
        self.last = 0.0

    def act(self, state, ranges, angles, dt):
        goal_yaw = self.goal_yaw(state)
        best_score = -1e9
        best_angle = 0.0
        for r, a in zip(ranges[::8], angles[::8]):
            candidate_yaw = state.yaw_deg - math.degrees(a)
            goal_err = math.radians(wrap_deg(candidate_yaw - goal_yaw))
            goal_score = math.cos(goal_err)
            clearance = clamp(r / MAX_RANGE, 0.0, 1.0)
            smooth = math.cos(a - self.last)
            score = 1.5 * goal_score + 1.55 * clearance + 0.16 * smooth
            if r < 2.8:
                score -= 6.0 * (2.8 - r)
            if score > best_score:
                best_score = score
                best_angle = a
        self.last = best_angle
        front = [r for r, a in zip(ranges, angles) if abs(a) < math.radians(20)]
        yaw_rate = clamp(-1.8 * math.degrees(best_angle), -60.0, 60.0)
        speed = 2.2 * clamp((min(front) - 0.25) / 5.5, 0.28, 1.0)
        return speed, yaw_rate


CONTROLLERS = {
    "modified_fly": ModifiedFlyController,
    "classical_vfh": VFHController,
    "no_avoidance": NoAvoidController,
}
async def wait_connected(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def wait_health(drone):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            return


async def wait_altitude(drone, target=1.8):
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= target:
            return float(pos.relative_altitude_m)


async def land_and_wait(drone):
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            return


def noisy_scan(ranges, rng, sigma=0.035, dropout=0.01):
    out = []
    for r in ranges:
        if rng.random() < dropout:
            out.append(MAX_RANGE)
        elif r < MAX_RANGE:
            out.append(clamp(r + rng.gauss(0.0, sigma), 0.1, MAX_RANGE))
        else:
            out.append(r)
    return out
async def run_trial(args):
    started = time.time()
    rng = random.Random(args.seed)
    controller = CONTROLLERS[args.controller]()
    controller.reset()
    lidar = LidarReader(args.container)
    print("STAGE lidar_start", flush=True)
    lidar.start()
    lidar.get(timeout=15.0)
    print("STAGE lidar_ready", flush=True)

    drone = System()
    print("STAGE mavsdk_connect", flush=True)
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    await asyncio.wait_for(wait_connected(drone), timeout=75)
    print("STAGE mavsdk_connected", flush=True)
    await asyncio.wait_for(wait_health(drone), timeout=90)
    print("STAGE health_ready", flush=True)

    state = FlightState()
    pos_task = asyncio.create_task(track_position(drone, state))
    att_task = asyncio.create_task(track_attitude(drone, state))

    await drone.action.set_takeoff_altitude(2.8)
    await drone.action.arm()
    print("STAGE takeoff", flush=True)
    await drone.action.takeoff()
    takeoff_alt = await asyncio.wait_for(wait_altitude(drone), timeout=35)
    print(f"STAGE takeoff_ready alt={takeoff_alt:.2f}", flush=True)
    await asyncio.sleep(1.0)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    print(f"OFFBOARD_STARTED controller={controller.name} altitude={takeoff_alt:.2f}")

    t0 = time.monotonic()
    last = t0
    trajectory = []
    min_clearance = MAX_RANGE
    success = False
    failure_reason = "timeout"
    next_log = t0
    try:
        while time.monotonic() - t0 < args.max_flight_s:
            ts, ranges, angles = lidar.get(timeout=4.0)
            now = time.monotonic()
            dt = max(now - last, 0.05)
            last = now
            ranges = noisy_scan(ranges, rng)
            finite = [r for r in ranges if r < MAX_RANGE]
            if finite:
                min_clearance = min(min_clearance, min(finite))

            speed, yaw_rate = controller.act(state, ranges, angles, dt)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(speed, 0.0, 0.0, yaw_rate)
            )
            trajectory.append({
                "t": round(now - t0, 3),
                "north": round(state.north, 3),
                "east": round(state.east, 3),
                "down": round(state.down, 3),
                "yaw_deg": round(state.yaw_deg, 2),
                "front_min_m": round(min(r for r, a in zip(ranges, angles)
                                           if abs(a) < math.radians(20)), 3),
                "speed_cmd_m_s": round(speed, 3),
                "yaw_rate_cmd_deg_s": round(yaw_rate, 2),
            })

            if now >= next_log:
                print(f"FLIGHT t={now-t0:.1f} n={state.north:.2f} e={state.east:.2f} min={min_clearance:.2f} cmd={speed:.2f}/{yaw_rate:.1f}", flush=True)
                next_log = now + 3.0

            if state.north >= GOAL_NORTH_M - 0.5:
                success = True
                failure_reason = ""
                print(f"GOAL_REACHED north={state.north:.2f} east={state.east:.2f}")
                break
            if state.down > -0.45 and now - t0 > 3.0:
                failure_reason = "unexpected_altitude_loss"
                break
            await asyncio.sleep(0.08)
    finally:
        await asyncio.wait_for(land_and_wait(drone), timeout=50)
        pos_task.cancel()
        att_task.cancel()
        lidar.stop()
    result = {
        "controller": controller.name,
        "trial": args.trial,
        "seed": args.seed,
        "success": success,
        "failure_reason": failure_reason,
        "duration_s": round(time.time() - started, 3),
        "flight_time_s": round(time.monotonic() - t0, 3),
        "final_north_m": round(state.north, 3),
        "final_east_m": round(state.east, 3),
        "min_clearance_m": round(min_clearance, 3),
        "trajectory": trajectory,
    }
    Path(args.out).write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "trajectory"}, indent=2))
    return result


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--controller", choices=sorted(CONTROLLERS), required=True)
    p.add_argument("--container", default="flydrone-final")
    p.add_argument("--trial", type=int, default=0)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--max-flight-s", type=float, default=34.0)
    p.add_argument("--out", required=True)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        asyncio.run(run_trial(args))
    except (OffboardError, Exception) as exc:
        failure = {
            "controller": args.controller,
            "trial": args.trial,
            "seed": args.seed,
            "success": False,
            "infra_error": type(exc).__name__,
            "message": str(exc),
        }
        Path(args.out).write_text(json.dumps(failure, indent=2), encoding="utf-8")
        print(json.dumps(failure, indent=2))
        raise
