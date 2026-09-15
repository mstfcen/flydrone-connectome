import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "prototype" / "3d"
summary = json.loads((ROOT / "summary3d.json").read_text())
stress = json.loads((ROOT / "stress_summary3d.json").read_text())

print("## Phase 2 — 3D benchmark\n")
print("3D inertial micro-UAV model; 105-ray retina. This is not yet full rigid-body propeller physics.\n")
print("### Held-out nominal benchmark\n")
print("| Controller | Sparse | Cluttered | Dense |")
print("|---|---:|---:|---:|")
for c, vals in summary.items():
    print(f"| `{c}` | {100*vals['sparse']['success_rate']:.1f}% | "
          f"{100*vals['cluttered']['success_rate']:.1f}% | "
          f"{100*vals['dense']['success_rate']:.1f}% |")

print("\n### Dense-map robustness\n")
print("| Controller | Clean | Sensor noise | 160 ms delay | Combined |")
print("|---|---:|---:|---:|---:|")
controllers = list(next(iter(stress.values())).keys())
for c in controllers:
    vals = [100*stress[p][c]['success_rate'] for p in
            ('clean','sensor_noise','delay_160ms','combined')]
    print(f"| `{c}` | {vals[0]:.1f}% | {vals[1]:.1f}% | {vals[2]:.1f}% | {vals[3]:.1f}% |")

print("\nRaw FlyWire and bilateralized FlyWire are intentionally reported separately. "
      "The bilateralized controller mirrors the better-connected left DNp03 circuit and is an engineering variant, not raw biology.")