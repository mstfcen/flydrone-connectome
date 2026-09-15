import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / "prototype" / "3d"
nom = json.loads((P / "summary6dof.json").read_text())
stress = json.loads((P / "stress_summary6dof.json").read_text())

names = [
    "fly_heuristic_6dof", "modified_fly_6dof", "flywire_raw_6dof",
    "flywire_bilateral_6dof", "classical_vfh_6dof",
]
diffs = ["sparse", "cluttered", "dense"]
profiles = ["clean", "sensor_noise", "latency_160ms", "gusty_wind", "combined"]

print("## Phase 3 — 6-DoF rigid-body quadcopter")
print()
print("Four motor thrust states, motor lag, gravity, attitude/body-rate dynamics, drag and wind.")
print()
print("### Held-out nominal benchmark")
print("| Controller | Sparse | Cluttered | Dense |")
print("|---|---:|---:|---:|")
for name in names:
    vals = [100 * nom[name][d]["success_rate"] for d in diffs]
    print(f"| `{name}` | {vals[0]:.1f}% | {vals[1]:.1f}% | {vals[2]:.1f}% |")

print()
print("### Dense-map robustness")
print("| Controller | Clean | Sensor noise | 160 ms delay | Gusty wind | Combined |")
print("|---|---:|---:|---:|---:|---:|")
for name in names:
    vals = [100 * stress[p][name]["success_rate"] for p in profiles]
    print(f"| `{name}` | {vals[0]:.1f}% | {vals[1]:.1f}% | {vals[2]:.1f}% | {vals[3]:.1f}% | {vals[4]:.1f}% |")

print()
print("Raw FlyWire and bilateralized FlyWire remain separate. The bilateralized variant mirrors the better-connected left DNp03 circuit and is an engineering intervention, not a raw-biological result.")
print()
print("This Phase-3 simulator adds 6-DoF rigid-body attitude/body-rate dynamics and four motor thrust states, but it is still a research simulator rather than PX4/Gazebo hardware-in-the-loop validation.")
