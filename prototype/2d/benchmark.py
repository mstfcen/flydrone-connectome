import csv
import json
import math
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

from sim import DT, make_demo_world, make_world, raycast, step
from controllers import FlyConnectomeController, ModifiedFlyController, VFHController

ROOT = Path(__file__).resolve().parent
MAX_STEPS = 650

def run_episode(controller, world, record=False):
    controller.reset()
    state = world.start.copy()
    start_dist = float(np.linalg.norm(state[:2] - world.goal))
    path_len = 0.0
    min_clearance = 99.0
    decision_times = []
    trajectory = [state.copy()]
    success = collision = False
    for k in range(MAX_STEPS):
        ranges, angles = raycast(state, world)
        min_clearance = min(min_clearance, float(np.min(ranges)))
        t0 = perf_counter()
        speed, omega = controller.act(state, ranges, angles, world.goal)
        decision_times.append((perf_counter() - t0) * 1e6)
        prev = state[:2].copy()
        state, collision, success = step(state, speed, omega, world)
        path_len += float(np.linalg.norm(state[:2] - prev))
        if record:
            trajectory.append(state.copy())
        if success or collision:
            break
    progress = 1.0 - float(np.linalg.norm(state[:2] - world.goal)) / start_dist
    return {
        "success": bool(success), "collision": bool(collision),
        "timeout": bool(not success and not collision), "steps": k + 1,
        "time_s": (k + 1) * DT, "path_m": path_len,
        "min_clearance_m": min_clearance, "progress": progress,
        "decision_us": float(np.mean(decision_times)),
        "trajectory": np.asarray(trajectory) if record else None,
    }

def tune_modified():
    rng = np.random.default_rng(12345)
    worlds = [make_world(4000 + i, "cluttered" if i % 2 == 0 else "dense")
              for i in range(16)]
    candidates = [ModifiedFlyController.defaults.copy()]
    for _ in range(35):
        candidates.append({
            "goal_gain": float(rng.uniform(0.8, 1.4)),
            "avoid_gain": float(rng.uniform(2.2, 4.2)),
            "ttc_gain": float(rng.uniform(0.9, 2.3)),
            "prox_gain": float(rng.uniform(0.35, 0.95)),
            "brake_gain": float(rng.uniform(0.55, 0.85)),
            "memory": float(rng.uniform(0.55, 0.9)),
        })
    best_score, best_params = -1e9, candidates[0]
    for params in candidates:
        scores = []
        for world in worlds:
            r = run_episode(ModifiedFlyController(params), world)
            scores.append(100*r["success"] - 70*r["collision"]
                          + 20*r["progress"] - 0.01*r["steps"])
        score = float(np.mean(scores))
        if score > best_score:
            best_score, best_params = score, params
    return best_params, best_score

def benchmark(best_params):
    factories = {
        "fly_connectome": FlyConnectomeController,
        "modified_fly": lambda: ModifiedFlyController(best_params),
        "classical_vfh": VFHController,
    }
    rows = []
    for difficulty in ("sparse", "cluttered", "dense"):
        worlds = [make_world(7000 + 1000*i + j, difficulty)
                  for i in range(1) for j in range(30)]
        for name, factory in factories.items():
            for world in worlds:
                r = run_episode(factory(), world)
                rows.append({"controller": name, "difficulty": difficulty,
                             "seed": world.seed, **{k:v for k,v in r.items()
                             if k != "trajectory"}})
    return rows

def summarize(rows):
    summary = {}
    for controller in sorted(set(r["controller"] for r in rows)):
        summary[controller] = {}
        for difficulty in ("sparse", "cluttered", "dense"):
            g = [r for r in rows if r["controller"] == controller
                 and r["difficulty"] == difficulty]
            summary[controller][difficulty] = {
                "success_rate": float(np.mean([r["success"] for r in g])),
                "collision_rate": float(np.mean([r["collision"] for r in g])),
                "mean_time_s": float(np.mean([r["time_s"] for r in g])),
                "mean_path_m": float(np.mean([r["path_m"] for r in g])),
                "mean_min_clearance_m": float(np.mean([r["min_clearance_m"] for r in g])),
                "mean_decision_us": float(np.mean([r["decision_us"] for r in g])),
            }
    return summary

def save_results(rows, summary, best_params, tune_score):
    cols = ["controller","difficulty","seed","success","collision","timeout",
            "steps","time_s","path_m","min_clearance_m","progress","decision_us"]
    with (ROOT / "results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    payload = {"modified_fly_params": best_params, "tune_score": tune_score,
               "summary": summary}
    (ROOT / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

def plot_success(summary):
    names = ["fly_connectome", "modified_fly", "classical_vfh"]
    diffs = ["sparse", "cluttered", "dense"]
    x = np.arange(len(diffs))
    width = 0.24
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    for i, name in enumerate(names):
        vals = [100*summary[name][d]["success_rate"] for d in diffs]
        ax.bar(x + (i-1)*width, vals, width, label=name)
    ax.set_xticks(x, diffs)
    ax.set_ylim(0, 105)
    ax.set_ylabel("Success rate (%)")
    ax.set_title("Held-out obstacle avoidance benchmark")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(ROOT / "success_rates.png", dpi=160)
    plt.close(fig)

def plot_demo(best_params):
    world = make_demo_world()
    controllers = [
        FlyConnectomeController(),
        ModifiedFlyController(best_params),
        VFHController(),
    ]
    fig, ax = plt.subplots(figsize=(10, 6))
    for o in world.obstacles:
        ax.add_patch(plt.Circle((o[0], o[1]), o[2], alpha=0.22))
    for c in controllers:
        r = run_episode(c, world, record=True)
        tr = r["trajectory"]
        ax.plot(tr[:,0], tr[:,1], linewidth=2, label=f"{c.name}: {'PASS' if r['success'] else 'FAIL'}")
    ax.scatter(*world.start[:2], marker="o", s=70, label="start")
    ax.scatter(*world.goal, marker="*", s=130, label="goal")
    ax.set_xlim(0, world.width); ax.set_ylim(0, world.height)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Same course, same 180° range retina, three controllers")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.grid(alpha=0.2); ax.legend(loc="upper center", ncol=2)
    fig.tight_layout()
    fig.savefig(ROOT / "demo_trajectories.png", dpi=160)
    plt.close(fig)

def print_summary(summary, best_params, tune_score):
    print("=== MODIFIED FLY TUNING ===")
    print(json.dumps(best_params, indent=2))
    print(f"training score: {tune_score:.2f}")
    print("\n=== HELD-OUT BENCHMARK (30 maps / difficulty) ===")
    print(f"{'controller':<18} {'difficulty':<11} {'success':>8} {'collision':>10} {'time_s':>8} {'decision_us':>12}")
    for name in ("fly_connectome","modified_fly","classical_vfh"):
        for d in ("sparse","cluttered","dense"):
            s = summary[name][d]
            print(f"{name:<18} {d:<11} {100*s['success_rate']:7.1f}% {100*s['collision_rate']:9.1f}% "
                  f"{s['mean_time_s']:8.2f} {s['mean_decision_us']:12.1f}")

def main():
    best_params, tune_score = tune_modified()
    rows = benchmark(best_params)
    summary = summarize(rows)
    save_results(rows, summary, best_params, tune_score)
    plot_success(summary)
    plot_demo(best_params)
    print_summary(summary, best_params, tune_score)

if __name__ == "__main__":
    main()
