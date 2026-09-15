# Phase 1 — 2D controller benchmark

This directory contains the planar baseline used to establish the first controlled comparison between bio-inspired and classical reactive avoidance policies.

All controllers receive the same 180° / 31-ray range observation and operate under identical kinematic limits:

- `fly_connectome` — hand-designed looming-reflex abstraction.
- `modified_fly` — TTC, proximity and temporal-memory extension.
- `classical_vfh` — VFH-style clearance/goal baseline.

## Reproduce

```bash
python benchmark.py
python stress_test.py
```

`benchmark.py` tunes `modified_fly`, evaluates 30 held-out maps at each of three difficulty levels, writes `results.csv` and `summary.json`, and regenerates the nominal plots.

`stress_test.py` reuses the tuned parameters and evaluates clean, sensor-noise/dropout, latency and combined stress profiles on dense maps. The checked-in CSV/JSON/PNG files are reference outputs; GitHub Actions reruns the same scripts independently.