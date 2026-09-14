# Original 2D controller benchmark

This directory contains the original Canavar simulation code and outputs from 2026-09-14.

Controllers are evaluated with the same 180-degree, 31-ray range retina and the same kinematic limits:

- `fly_connectome` — hand-designed fly-inspired looming reflex.
- `modified_fly` — fly-inspired reflex plus TTC, proximity and temporal memory.
- `classical_vfh` — classical VFH-style clearance/goal controller.

## Reproduce

```bash
python benchmark.py
python stress_test.py
```

`benchmark.py` tunes `modified_fly`, runs 30 held-out maps at each of three difficulty levels, writes `results.csv` + `summary.json`, and regenerates the nominal plots. `stress_test.py` reuses the tuned parameters and evaluates clean, sensor-noise/dropout, latency, and combined profiles on dense maps.

The checked-in CSV/JSON/PNG files are the original outputs transferred directly from Canavar. GitHub Actions reruns the same scripts and uploads fresh artifacts for each CI run.
