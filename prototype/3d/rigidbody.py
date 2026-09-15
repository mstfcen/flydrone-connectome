import math
from dataclasses import dataclass
import numpy as np

from sim3d import (
    AZIMUTHS, ELEVATIONS, RAY_AZ, RAY_EL, MAX_RANGE,
    DRONE_RADIUS, World3D, make_world, make_demo_world, wrap,
)

PHYS_DT = 0.02
CONTROL_DT = 0.08
CONTROL_STEPS = int(round(CONTROL_DT / PHYS_DT))
MASS = 1.0
GRAVITY = 9.81
INERTIA = np.array([0.018, 0.018, 0.032])
ARM = 0.18 / math.sqrt(2.0)
YAW_TORQUE_PER_N = 0.020
MOTOR_TAU = 0.065
MAX_MOTOR_THRUST = 7.5
LINEAR_DRAG = 0.18
ANGULAR_DRAG = np.array([0.025, 0.025, 0.035])
MAX_TILT = math.radians(35.0)
MAX_MOMENT = np.array([0.55, 0.55, 0.22])
HOVER_THRUST = MASS * GRAVITY / 4.0

# X-quad mixer: motor order FL, FR, RR, RL.
_MIX = np.array([
    [1.0, 1.0, 1.0, 1.0],
    [ARM, -ARM, -ARM, ARM],
    [-ARM, -ARM, ARM, ARM],
    [YAW_TORQUE_PER_N, -YAW_TORQUE_PER_N, YAW_TORQUE_PER_N, -YAW_TORQUE_PER_N],
])
_MIX_INV = np.linalg.inv(_MIX)


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


def make_rigid_state(world):
    state = np.zeros(16, dtype=float)
    state[:3] = world.start[:3]
    state[8] = world.start[6]
    state[12:16] = HOVER_THRUST
    return state

def controller_state(state):
    # Compatibility view for Phase-2 high-level controllers:
    # x,y,z,vx,vy,vz,yaw
    return np.r_[state[:6], state[8]]


def ray_directions_attitude(roll, pitch, yaw):
    ca, sa = np.cos(RAY_AZ), np.sin(RAY_AZ)
    ce, se = np.cos(RAY_EL), np.sin(RAY_EL)
    local = np.column_stack((ce * ca, ce * sa, se))
    R = rotation_matrix(roll, pitch, yaw)
    return local @ R.T


def raycast_rigid(state, world):
    p = state[:3]
    dirs = ray_directions_attitude(*state[6:9])
    best = np.full(len(dirs), MAX_RANGE, dtype=float)
    if len(world.obstacles):
        oc = p[None, None, :] - world.obstacles[None, :, :3]
        d = dirs[:, None, :]
        b = 2.0 * np.sum(d * oc, axis=2)
        c = np.sum(oc * oc, axis=2) - (world.obstacles[None, :, 3] + DRONE_RADIUS) ** 2
        disc = b*b - 4.0*c
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


def high_level_to_motor_targets(state, command):
    speed_cmd, yaw_rate_cmd, climb_cmd = map(float, command)
    roll, pitch, yaw = state[6:9]
    p_rate, q_rate, r_rate = state[9:12]
    vel = state[3:6]
    desired_v = np.array([
        speed_cmd * math.cos(yaw),
        speed_cmd * math.sin(yaw),
        climb_cmd,
    ])
    a_cmd = 1.8 * (desired_v - vel)
    a_cmd[:2] = np.clip(a_cmd[:2], -5.0, 5.0)
    a_cmd[2] = float(np.clip(a_cmd[2], -4.0, 4.0))
    ax, ay = a_cmd[0], a_cmd[1]
    roll_des = (math.sin(yaw) * ax - math.cos(yaw) * ay) / GRAVITY
    pitch_des = (math.cos(yaw) * ax + math.sin(yaw) * ay) / GRAVITY
    roll_des = float(np.clip(roll_des, -MAX_TILT, MAX_TILT))
    pitch_des = float(np.clip(pitch_des, -MAX_TILT, MAX_TILT))

    collective = MASS * (GRAVITY + a_cmd[2]) / max(math.cos(roll) * math.cos(pitch), 0.45)
    collective = float(np.clip(collective, 0.15 * MASS * GRAVITY, 4.0 * MAX_MOTOR_THRUST))

    moments = np.array([
        0.30 * (roll_des - roll) - 0.075 * p_rate,
        0.30 * (pitch_des - pitch) - 0.075 * q_rate,
        0.11 * (yaw_rate_cmd - r_rate),
    ])
    moments = np.clip(moments, -MAX_MOMENT, MAX_MOMENT)
    wrench = np.r_[collective, moments]
    motor_targets = _MIX_INV @ wrench
    return np.clip(motor_targets, 0.0, MAX_MOTOR_THRUST)


def _euler_rates(euler, omega):
    roll, pitch, _ = euler
    p, q, r = omega
    cp = max(abs(math.cos(pitch)), 0.08)
    tp = math.sin(pitch) / cp
    sr, cr = math.sin(roll), math.cos(roll)
    return np.array([
        p + q * sr * tp + r * cr * tp,
        q * cr - r * sr,
        q * sr / cp + r * cr / cp,
    ])


def wind_vector(world, sim_time, profile="calm"):
    if profile == "calm":
        return np.zeros(3)
    rng_phase = (world.seed % 997) * 0.017
    if profile == "breeze":
        base, gust = np.array([1.0, -0.4, 0.0]), 0.8
    else:
        base, gust = np.array([2.0, -0.8, 0.2]), 1.8
    return base + gust * np.array([
        math.sin(0.63 * sim_time + rng_phase),
        0.7 * math.cos(0.47 * sim_time + 0.7 * rng_phase),
        0.25 * math.sin(0.91 * sim_time + 1.3 * rng_phase),
    ])


def physics_step(state, motor_targets, world, sim_time=0.0, wind_profile="calm"):
    s = state.copy()
    motors = s[12:16]
    motors += (np.asarray(motor_targets) - motors) * (PHYS_DT / MOTOR_TAU)
    motors[:] = np.clip(motors, 0.0, MAX_MOTOR_THRUST)
    total_thrust = float(np.sum(motors))
    R = rotation_matrix(*s[6:9])
    thrust_world = R @ np.array([0.0, 0.0, total_thrust])
    wind = wind_vector(world, sim_time, wind_profile)
    drag_force = -LINEAR_DRAG * (s[3:6] - wind)
    accel = thrust_world / MASS + drag_force / MASS + np.array([0.0, 0.0, -GRAVITY])
    s[3:6] += accel * PHYS_DT
    s[:3] += s[3:6] * PHYS_DT

    wrench = _MIX @ motors
    moments = wrench[1:4]
    omega = s[9:12]
    coriolis = np.cross(omega, INERTIA * omega)
    omega_dot = (moments - coriolis - ANGULAR_DRAG * omega) / INERTIA
    s[9:12] += omega_dot * PHYS_DT
    s[6:9] += _euler_rates(s[6:9], s[9:12]) * PHYS_DT
    s[6] = wrap(s[6])
    s[7] = float(np.clip(s[7], -math.radians(85.0), math.radians(85.0)))
    s[8] = wrap(s[8])

    pos = s[:3]
    collision = bool(np.any(pos < DRONE_RADIUS) or np.any(pos > world.size - DRONE_RADIUS))
    if len(world.obstacles):
        d = np.linalg.norm(world.obstacles[:, :3] - pos, axis=1)
        collision = collision or bool(np.any(d <= world.obstacles[:, 3] + DRONE_RADIUS))
    unstable = abs(s[6]) > math.radians(78.0) or abs(s[7]) > math.radians(78.0)
    collision = collision or unstable
    success = bool(np.linalg.norm(pos - world.goal) <= 1.5)
    return s, collision, success
