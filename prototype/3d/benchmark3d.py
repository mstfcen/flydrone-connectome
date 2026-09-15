import csv
import json
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

from sim3d import DT, make_demo_world, make_world, raycast, step
from controllers3d import (
    Fly3DController, ModifiedFly3DController,
    FlyWireGraph3DController, BilateralFlyWire3DController, VFH3DController,
)

ROOT = Path(__file__).resolve().parent
MAX_STEPS = 850
MAPS_PER_DIFFICULTY = 16


def run_episode(controller, world, record=False, noise=0.0, dropout=0.0, delay_steps=0):
    controller.reset()
    state = world.start.copy()
    start_dist = float(np.linalg.norm(state[:3] - world.goal))
    rng = np.random.default_rng(world.seed + 991)
    queue = [(0.0, 0.0, 0.0)] * delay_steps
    path_len = 0.0
    min_range = 99.0
    decision_times = []
    trajectory = [state.copy()]
    success = collision = False
    for k in range(MAX_STEPS):
        ranges, az, el = raycast(state, world)
        min_range = min(min_range, float(np.min(ranges)))
        sensed = ranges.copy()
        if noise:
            sensed += rng.normal(0.0, noise, size=sensed.shape)
            sensed = np.clip(sensed, 0.01, 15.0)
        if dropout:
            sensed[rng.random(len(sensed)) < dropout] = 15.0
        t0 = perf_counter()
        action = controller.act(state, sensed, az, el, world.goal)
        decision_times.append((perf_counter() - t0) * 1e6)
        if delay_steps:
            queue.append(action)
            action = queue.pop(0)
        prev = state[:3].copy()
        state, collision, success = step(state, *action, world)
        path_len += float(np.linalg.norm(state[:3] - prev))
        if record:
            trajectory.append(state.copy())
        if success or collision:
            break
    progress = 1.0 - float(np.linalg.norm(state[:3] - world.goal)) / start_dist
    return {
        "success": bool(success), "collision": bool(collision),
        "timeout": bool(not success and not collision), "steps": k + 1,
        "time_s": (k + 1) * DT, "path_m": path_len,
        "min_range_m": min_range, "progress": progress,
        "decision_us": float(np.mean(decision_times)),
        "trajectory": np.asarray(trajectory) if record else None,
    }


def factories():
    return {
        "fly_heuristic_3d": Fly3DController,
        "modified_fly_3d": ModifiedFly3DController,
        "flywire_graph_3d": FlyWireGraph3DController,
        "flywire_bilateral_3d": BilateralFlyWire3DController,
        "classical_vfh_3d": VFH3DController,
    }

def benchmark():
    rows = []
    for difficulty in ("sparse", "cluttered", "dense"):
        worlds = [make_world(12000 + 1000*i + j, difficulty)
                  for i in range(1) for j in range(MAPS_PER_DIFFICULTY)]
        for name, factory in factories().items():
            controller = factory()
            for world in worlds:
                r = run_episode(controller, world)
                rows.append({"controller": name, "difficulty": difficulty,
                             "seed": world.seed,
                             **{k:v for k,v in r.items() if k != "trajectory"}})
    return rows


def summarize(rows):
    summary = {}
    for controller in factories():
        summary[controller] = {}
        for difficulty in ("sparse", "cluttered", "dense"):
            g = [r for r in rows if r["controller"] == controller
                 and r["difficulty"] == difficulty]
            summary[controller][difficulty] = {
                "success_rate": float(np.mean([r["success"] for r in g])),
                "collision_rate": float(np.mean([r["collision"] for r in g])),
                "mean_time_s": float(np.mean([r["time_s"] for r in g])),
                "mean_path_m": float(np.mean([r["path_m"] for r in g])),
                "mean_min_range_m": float(np.mean([r["min_range_m"] for r in g])),
                "mean_decision_us": float(np.mean([r["decision_us"] for r in g])),
            }
    return summary


def save(rows, summary):
    cols = ["controller","difficulty","seed","success","collision","timeout",
            "steps","time_s","path_m","min_range_m","progress","decision_us"]
    with (ROOT / "results3d.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader(); w.writerows(rows)
    (ROOT / "summary3d.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

def plot_success(summary):
    names = list(factories())
    diffs = ["sparse", "cluttered", "dense"]
    x = np.arange(len(diffs)); width = 0.16
    fig, ax = plt.subplots(figsize=(9.5, 5.0))
    for i, name in enumerate(names):
        vals = [100 * summary[name][d]["success_rate"] for d in diffs]
        ax.bar(x + (i-(len(names)-1)/2)*width, vals, width, label=name)
    ax.set_xticks(x, diffs); ax.set_ylim(0, 105)
    ax.set_ylabel("Success rate (%)")
    ax.set_title("3D held-out obstacle avoidance benchmark")
    ax.grid(axis="y", alpha=0.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(ROOT / "success_rates_3d.png", dpi=160)
    plt.close(fig)


def plot_demo():
    world = make_demo_world()
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    for name, factory in factories().items():
        r = run_episode(factory(), world, record=True)
        tr = r["trajectory"]
        label = f"{name}: {'PASS' if r['success'] else 'FAIL'}"
        ax.plot(tr[:,0], tr[:,1], tr[:,2], linewidth=2, label=label)
    ax.scatter(world.obstacles[:,0], world.obstacles[:,1], world.obstacles[:,2],
               s=(world.obstacles[:,3] * 35) ** 2, alpha=0.12)
    ax.scatter(*world.start[:3], s=60, marker="o", label="start")
    ax.scatter(*world.goal, s=120, marker="*", label="goal")
    ax.set(xlim=(0,world.size[0]), ylim=(0,world.size[1]), zlim=(0,world.size[2]),
           xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    ax.set_title("Same 3D course, same 105-ray retina")
    ax.legend(fontsize=8); fig.tight_layout()
    fig.savefig(ROOT / "demo_trajectories_3d.png", dpi=160)
    plt.close(fig)

def print_summary(summary):
    print(f"=== 3D BENCHMARK ({MAPS_PER_DIFFICULTY} maps / difficulty) ===")
    print(f"{'controller':<20} {'difficulty':<11} {'success':>8} {'collision':>10} {'time_s':>8} {'decision_us':>12}")
    for name in factories():
        for d in ("sparse", "cluttered", "dense"):
            s = summary[name][d]
            print(f"{name:<20} {d:<11} {100*s['success_rate']:7.1f}% "
                  f"{100*s['collision_rate']:9.1f}% {s['mean_time_s']:8.2f} "
                  f"{s['mean_decision_us']:12.1f}")


def main():
    rows = benchmark()
    summary = summarize(rows)
    save(rows, summary)
    plot_success(summary)
    plot_demo()
    print_summary(summary)


if __name__ == "__main__":
    main()