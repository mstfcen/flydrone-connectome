import json
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from sim import DT, MAX_RANGE, make_world, raycast, step
from controllers import FlyConnectomeController, ModifiedFlyController, VFHController

ROOT = Path(__file__).resolve().parent
MAX_STEPS = 650

PROFILES = {
    "clean": dict(noise=0.0, dropout=0.0, delay=0),
    "sensor_noise": dict(noise=0.20, dropout=0.03, delay=0),
    "delay_160ms": dict(noise=0.0, dropout=0.0, delay=2),
    "combined": dict(noise=0.20, dropout=0.03, delay=2),
}

def run_episode(controller, world, profile):
    controller.reset()
    state = world.start.copy()
    rng = np.random.default_rng(world.seed + 123)
    queue = deque([(0.0, 0.0)] * profile["delay"])
    success = collision = False
    for k in range(MAX_STEPS):
        ranges, angles = raycast(state, world)
        sensed = ranges.copy()
        if profile["noise"] > 0:
            sensed += rng.normal(0.0, profile["noise"], size=sensed.shape)
            sensed = np.clip(sensed, 0.01, MAX_RANGE)
        if profile["dropout"] > 0:
            mask = rng.random(len(sensed)) < profile["dropout"]
            sensed[mask] = MAX_RANGE
        action = controller.act(state, sensed, angles, world.goal)
        if profile["delay"]:
            queue.append(action)
            speed, omega = queue.popleft()
        else:
            speed, omega = action
        state, collision, success = step(state, speed, omega, world)
        if success or collision:
            break
    return {"success": bool(success), "collision": bool(collision),
            "timeout": bool(not success and not collision),
            "time_s": (k + 1) * DT}

def main():
    prior = json.loads((ROOT / "summary.json").read_text(encoding="utf-8"))
    params = prior["modified_fly_params"]
    factories = {
        "fly_connectome": FlyConnectomeController,
        "modified_fly": lambda: ModifiedFlyController(params),
        "classical_vfh": VFHController,
    }
    worlds = [make_world(9000 + j, "dense") for j in range(30)]
    summary = {}
    for pname, profile in PROFILES.items():
        summary[pname] = {}
        for cname, factory in factories.items():
            rows = [run_episode(factory(), world, profile) for world in worlds]
            summary[pname][cname] = {
                "success_rate": float(np.mean([r["success"] for r in rows])),
                "collision_rate": float(np.mean([r["collision"] for r in rows])),
                "mean_time_s": float(np.mean([r["time_s"] for r in rows])),
            }
    (ROOT / "stress_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    names = list(factories)
    x = np.arange(len(PROFILES))
    width = 0.24
    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    for i, name in enumerate(names):
        vals = [100 * summary[p][name]["success_rate"] for p in PROFILES]
        ax.bar(x + (i-1)*width, vals, width, label=name)
    ax.set_xticks(x, list(PROFILES))
    ax.set_ylim(0, 105)
    ax.set_ylabel("Success rate (%)")
    ax.set_title("Dense-map robustness: noise, dropout and control latency")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(ROOT / "stress_success.png", dpi=160)
    plt.close(fig)
    print("=== DENSE ROBUSTNESS TEST: 30 MAPS / PROFILE ===")
    print(f"{'profile':<15} {'controller':<18} {'success':>8} {'collision':>10} {'time_s':>8}")
    for p in PROFILES:
        for c in names:
            s = summary[p][c]
            print(f"{p:<15} {c:<18} {100*s['success_rate']:7.1f}% "
                  f"{100*s['collision_rate']:9.1f}% {s['mean_time_s']:8.2f}")

if __name__ == "__main__":
    main()
