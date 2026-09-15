import csv
import json
import math
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

from sim3d import make_world, make_demo_world, MAX_RANGE
from controllers3d import (
    Fly3DController, ModifiedFly3DController, FlyWireGraph3DController,
    BilateralFlyWire3DController, VFH3DController,
)
from rigidbody import (
    PHYS_DT, CONTROL_STEPS, HOVER_THRUST, MAX_MOTOR_THRUST,
    make_rigid_state, controller_state, raycast_rigid,
    high_level_to_motor_targets, physics_step,
)

ROOT = Path(__file__).resolve().parent
MAX_TIME = 75.0
MAPS_PER_DIFFICULTY = 12
CONTROLLERS = {
    "fly_heuristic_6dof": Fly3DController,
    "modified_fly_6dof": ModifiedFly3DController,
    "flywire_raw_6dof": FlyWireGraph3DController,
    "flywire_bilateral_6dof": BilateralFlyWire3DController,
    "classical_vfh_6dof": VFH3DController,
}

def run_episode(controller, world, record=False, noise=0.0, dropout=0.0,
                delay_frames=0, wind_profile="calm"):
    controller.reset()
    state = make_rigid_state(world)
    motor_targets = np.full(4, HOVER_THRUST)
    rng = np.random.default_rng(world.seed + 20260915)
    command_queue = [(0.0, 0.0, 0.0)] * delay_frames
    start_dist = float(np.linalg.norm(world.start[:3] - world.goal))
    path_m = 0.0
    min_range_m = MAX_RANGE
    max_tilt = 0.0
    sat_steps = 0
    decision_us = []
    motor_effort = 0.0
    trajectory = [state.copy()]
    success = collision = False
    prev_pos = state[:3].copy()
    n_steps = int(MAX_TIME / PHYS_DT)

    for k in range(n_steps):
        if k % CONTROL_STEPS == 0:
            ranges, az, el = raycast_rigid(state, world)
            min_range_m = min(min_range_m, float(np.min(ranges)))
            sensed = ranges.copy()
            if noise:
                sensed += rng.normal(0.0, noise, sensed.shape)
                sensed = np.clip(sensed, 0.01, MAX_RANGE)
            if dropout:
                sensed[rng.random(len(sensed)) < dropout] = MAX_RANGE
            t0 = perf_counter()
            cmd = controller.act(controller_state(state), sensed, az, el, world.goal)
            decision_us.append((perf_counter() - t0) * 1e6)
            if delay_frames:
                command_queue.append(cmd)
                cmd = command_queue.pop(0)
            motor_targets = high_level_to_motor_targets(state, cmd)

        sat_steps += int(np.any((motor_targets <= 1e-6) |
                                (motor_targets >= MAX_MOTOR_THRUST - 1e-6)))
        motor_effort += float(np.sum(np.power(state[12:16], 1.5))) * PHYS_DT
        state, collision, success = physics_step(
            state, motor_targets, world, k * PHYS_DT, wind_profile)
        path_m += float(np.linalg.norm(state[:3] - prev_pos))
        prev_pos = state[:3].copy()
        max_tilt = max(max_tilt, abs(state[6]), abs(state[7]))
        if record and k % CONTROL_STEPS == 0:
            trajectory.append(state.copy())
        if collision or success:
            break

    progress = 1.0 - float(np.linalg.norm(state[:3] - world.goal)) / start_dist
    return {
        "success": bool(success), "collision": bool(collision),
        "timeout": bool(not success and not collision), "steps": k + 1,
        "time_s": (k + 1) * PHYS_DT, "path_m": path_m,
        "min_range_m": min_range_m, "progress": progress,
        "max_tilt_deg": math.degrees(max_tilt),
        "motor_saturation_frac": sat_steps / (k + 1),
        "motor_effort": motor_effort,
        "decision_us": float(np.mean(decision_us)) if decision_us else 0.0,
        "trajectory": np.asarray(trajectory) if record else None,
    }

def benchmark():
    rows = []
    for difficulty in ("sparse", "cluttered", "dense"):
        worlds = [make_world(15000 + 1000*i + j, difficulty)
                  for i in range(1) for j in range(MAPS_PER_DIFFICULTY)]
        for name, factory in CONTROLLERS.items():
            for world in worlds:
                r = run_episode(factory(), world)
                rows.append({"controller": name, "difficulty": difficulty,
                             "seed": world.seed,
                             **{k:v for k,v in r.items() if k != "trajectory"}})
    return rows


def summarize(rows):
    out = {}
    for name in CONTROLLERS:
        out[name] = {}
        for difficulty in ("sparse", "cluttered", "dense"):
            g = [r for r in rows if r["controller"] == name and r["difficulty"] == difficulty]
            out[name][difficulty] = {
                "success_rate": float(np.mean([r["success"] for r in g])),
                "collision_rate": float(np.mean([r["collision"] for r in g])),
                "mean_time_s": float(np.mean([r["time_s"] for r in g])),
                "mean_max_tilt_deg": float(np.mean([r["max_tilt_deg"] for r in g])),
                "mean_motor_saturation_frac": float(np.mean([r["motor_saturation_frac"] for r in g])),
                "mean_motor_effort": float(np.mean([r["motor_effort"] for r in g])),
                "mean_decision_us": float(np.mean([r["decision_us"] for r in g])),
            }
    return out

def save_outputs(rows, summary):
    cols = ["controller","difficulty","seed","success","collision","timeout",
            "steps","time_s","path_m","min_range_m","progress","max_tilt_deg",
            "motor_saturation_frac","motor_effort","decision_us"]
    with (ROOT / "results6dof.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader(); w.writerows(rows)
    (ROOT / "summary6dof.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    diffs = ["sparse", "cluttered", "dense"]
    x = np.arange(len(diffs)); width = 0.16
    fig, ax = plt.subplots(figsize=(10.0, 5.0))
    for i, name in enumerate(CONTROLLERS):
        vals = [100 * summary[name][d]["success_rate"] for d in diffs]
        ax.bar(x + (i-2)*width, vals, width, label=name)
    ax.set_xticks(x, diffs); ax.set_ylim(0,105); ax.set_ylabel("Success rate (%)")
    ax.set_title("Phase 3 — 6-DoF rigid-body quadcopter benchmark")
    ax.grid(axis="y", alpha=.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(ROOT / "success_rates_6dof.png", dpi=160); plt.close(fig)


def plot_demo():
    world = make_demo_world()
    fig = plt.figure(figsize=(10.5, 7.0)); ax = fig.add_subplot(111, projection="3d")
    u = np.linspace(0.0, 2.0*np.pi, 12)
    v = np.linspace(0.0, np.pi, 8)
    for ox, oy, oz, radius in world.obstacles:
        xs = ox + radius * np.outer(np.cos(u), np.sin(v))
        ys = oy + radius * np.outer(np.sin(u), np.sin(v))
        zs = oz + radius * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(xs, ys, zs, alpha=0.08, linewidth=0)
    ax.scatter([], [], [], s=35, alpha=0.18, label="obstacles")
    for name, factory in CONTROLLERS.items():
        r = run_episode(factory(), world, record=True)
        tr = r["trajectory"]
        ax.plot(tr[:,0], tr[:,1], tr[:,2], linewidth=2,
                label=f"{name}: {'PASS' if r['success'] else 'FAIL'}")
    ax.scatter(*world.start[:3], s=60, label="start")
    ax.scatter(*world.goal, marker="*", s=120, label="goal")
    ax.set_xlim(0, world.size[0]); ax.set_ylim(0, world.size[1]); ax.set_zlim(0, world.size[2])
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title("Same 3D course through 6-DoF motor + attitude dynamics")
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(ROOT / "demo_trajectories_6dof.png", dpi=160); plt.close(fig)


def print_summary(summary):
    print(f"=== PHASE 3 — 6-DoF ({MAPS_PER_DIFFICULTY} maps / difficulty) ===")
    print(f"{'controller':<26} {'difficulty':<11} {'success':>8} {'collision':>10} {'time_s':>8} {'tilt':>7} {'sat':>7}")
    for name in CONTROLLERS:
        for d in ("sparse", "cluttered", "dense"):
            s = summary[name][d]
            print(f"{name:<26} {d:<11} {100*s['success_rate']:7.1f}% "
                  f"{100*s['collision_rate']:9.1f}% {s['mean_time_s']:8.2f} "
                  f"{s['mean_max_tilt_deg']:6.1f}° {100*s['mean_motor_saturation_frac']:6.1f}%")


def main():
    rows = benchmark()
    summary = summarize(rows)
    save_outputs(rows, summary)
    plot_demo()
    print_summary(summary)


if __name__ == "__main__":
    main()
