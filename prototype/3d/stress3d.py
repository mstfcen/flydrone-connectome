import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from benchmark3d import factories, run_episode
from sim3d import make_world

ROOT = Path(__file__).resolve().parent
MAPS = 16
PROFILES = {
    "clean": dict(noise=0.0, dropout=0.0, delay_steps=0),
    "sensor_noise": dict(noise=0.20, dropout=0.03, delay_steps=0),
    "delay_160ms": dict(noise=0.0, dropout=0.0, delay_steps=2),
    "combined": dict(noise=0.20, dropout=0.03, delay_steps=2),
}


def main():
    worlds = [make_world(15000 + j, "dense") for j in range(MAPS)]
    summary = {}
    for pname, profile in PROFILES.items():
        summary[pname] = {}
        for cname, factory in factories().items():
            rows = [run_episode(factory(), world, **profile) for world in worlds]
            summary[pname][cname] = {
                "success_rate": float(np.mean([r["success"] for r in rows])),
                "collision_rate": float(np.mean([r["collision"] for r in rows])),
                "mean_time_s": float(np.mean([r["time_s"] for r in rows])),
            }
    (ROOT / "stress_summary3d.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    names = list(factories())
    x = np.arange(len(PROFILES)); width = 0.16
    fig, ax = plt.subplots(figsize=(10.5, 5.0))
    for i, name in enumerate(names):
        vals = [100 * summary[p][name]["success_rate"] for p in PROFILES]
        ax.bar(x + (i-(len(names)-1)/2)*width, vals, width, label=name)
    ax.set_xticks(x, list(PROFILES)); ax.set_ylim(0, 105)
    ax.set_ylabel("Success rate (%)")
    ax.set_title("3D dense-map robustness: sensor noise, dropout and latency")
    ax.grid(axis="y", alpha=0.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(ROOT / "stress_success_3d.png", dpi=160)
    plt.close(fig)

    print(f"=== 3D DENSE ROBUSTNESS ({MAPS} maps/profile) ===")
    print(f"{'profile':<15} {'controller':<22} {'success':>8} {'collision':>10} {'time_s':>8}")
    for p in PROFILES:
        for c in names:
            s = summary[p][c]
            print(f"{p:<15} {c:<22} {100*s['success_rate']:7.1f}% "
                  f"{100*s['collision_rate']:9.1f}% {s['mean_time_s']:8.2f}")


if __name__ == "__main__":
    main()