import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from sim3d import make_world
from controllers3d import (
    Fly3DController, ModifiedFly3DController, FlyWireGraph3DController,
    BilateralFlyWire3DController, VFH3DController,
)
from benchmark6dof import run_episode

ROOT = Path(__file__).resolve().parent
MAPS = 10
FACTORIES = {
    "fly_heuristic_6dof": Fly3DController,
    "modified_fly_6dof": ModifiedFly3DController,
    "flywire_raw_6dof": FlyWireGraph3DController,
    "flywire_bilateral_6dof": BilateralFlyWire3DController,
    "classical_vfh_6dof": VFH3DController,
}
PROFILES = {
    "clean": dict(noise=0.0, dropout=0.0, delay_frames=0, wind_profile="calm"),
    "sensor_noise": dict(noise=0.25, dropout=0.04, delay_frames=0, wind_profile="calm"),
    "latency_160ms": dict(noise=0.0, dropout=0.0, delay_frames=2, wind_profile="calm"),
    "gusty_wind": dict(noise=0.0, dropout=0.0, delay_frames=0, wind_profile="gusty"),
    "combined": dict(noise=0.25, dropout=0.04, delay_frames=2, wind_profile="gusty"),
}

def main():
    worlds = [make_world(19000 + j, "dense") for j in range(MAPS)]
    summary = {}
    for pname, profile in PROFILES.items():
        summary[pname] = {}
        for cname, factory in FACTORIES.items():
            rows = [run_episode(factory(), world, **profile) for world in worlds]
            summary[pname][cname] = {
                "success_rate": float(np.mean([r["success"] for r in rows])),
                "collision_rate": float(np.mean([r["collision"] for r in rows])),
                "mean_time_s": float(np.mean([r["time_s"] for r in rows])),
                "mean_max_tilt_deg": float(np.mean([r["max_tilt_deg"] for r in rows])),
                "mean_motor_saturation_frac": float(np.mean([r["motor_saturation_frac"] for r in rows])),
            }
    (ROOT / "stress_summary6dof.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")

    names = list(FACTORIES)
    x = np.arange(len(PROFILES)); width = 0.16
    fig, ax = plt.subplots(figsize=(11.0, 5.2))
    for i, name in enumerate(names):
        vals = [100 * summary[p][name]["success_rate"] for p in PROFILES]
        ax.bar(x + (i-2)*width, vals, width, label=name)
    ax.set_xticks(x, list(PROFILES)); ax.set_ylim(0,105)
    ax.set_ylabel("Success rate (%)")
    ax.set_title("Phase 3 — 6-DoF robustness: sensing, latency and gusty wind")
    ax.grid(axis="y", alpha=.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(ROOT / "stress_success_6dof.png", dpi=160); plt.close(fig)

    print(f"=== PHASE 3 — 6-DoF DENSE ROBUSTNESS ({MAPS} maps/profile) ===")
    print(f"{'profile':<16} {'controller':<26} {'success':>8} {'collision':>10} {'time_s':>8} {'tilt':>7} {'sat':>7}")
    for pname in PROFILES:
        for cname in names:
            s = summary[pname][cname]
            print(f"{pname:<16} {cname:<26} {100*s['success_rate']:7.1f}% "
                  f"{100*s['collision_rate']:9.1f}% {s['mean_time_s']:8.2f} "
                  f"{s['mean_max_tilt_deg']:6.1f}° {100*s['mean_motor_saturation_frac']:6.1f}%")


if __name__ == "__main__":
    main()
