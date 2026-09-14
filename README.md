# FlyDrone Connectome Prototype

[![FlyDrone tests](https://github.com/mstfcen/flydrone-connectome/actions/workflows/tests.yml/badge.svg)](https://github.com/mstfcen/flydrone-connectome/actions/workflows/tests.yml)

Bio-inspired UAV obstacle-avoidance research prototype comparing a fly-like looming reflex, a modified temporal-memory fly controller, and a classical VFH-style controller. The project has now begun replacing the hand-designed biological abstraction with a real FlyWire FAFB v783 subgraph.

## Current 2D results

Nominal held-out benchmark: 90 maps total (30 sparse, 30 cluttered, 30 dense). Overall success was ~76.7% for the fly-reflex controller, 90.0% for modified-fly, and 100% for classical VFH. In dense scenes modified-fly reached 30/30.

Robustness test (dense maps, 30 runs/profile): with range noise + 3% ray dropout, success was 90.0% fly-reflex, 86.7% modified-fly, 46.7% VFH. With noise/dropout + 160 ms control delay, success was 86.7%, 93.3%, and 50.0% respectively.

These are prototype simulation results, not evidence that a biological controller is generally superior. The noise result is hypothesis-generating and needs larger reruns and stronger baselines.

### Original Canavar figures

![Same-course controller trajectories](prototype/2d/demo_trajectories.png)

![Held-out success rates](prototype/2d/success_rates.png)

![Noise and latency robustness](prototype/2d/stress_success.png)

## Tests and reproducibility

The original Canavar source, raw episode table and figures now live in [`prototype/2d/`](prototype/2d/). The checked-in outputs were transferred directly from Canavar rather than reconstructed from summaries.

GitHub Actions runs two jobs on every push/PR:

1. **Smoke tests** — deterministic world generation, ray sensor contract, all controller action contracts, and a known dense-map success case.
2. **Full benchmark + robustness** — reruns `benchmark.py` and `stress_test.py`, then uploads CSV, JSON, PNG figures, and console logs as workflow artifacts.

Open the **Actions** tab or click the badge above to inspect each run.

## Real connectome extraction

The Zaku mirror contains a compact circuit extracted from FlyWire FAFB v783: 104 LC4, 210 LPLC2 and 2 DNp03 neurons. In this snapshot the extraction found 317 direct LC4/LPLC2-to-DNp03 synapses over 37 directed edges, plus 61 two-hop intermediates. The exported compact circuit contains 377 selected neurons and 1,117 selected edges.

See `out/fafb_loom_dnp03_circuit.json` and `out/fafb_loom_dnp03_report.json`.

## Repository map

- `prototype/2d/` — original Canavar simulation source, episode-level results and figures
- `tests/` — fast CI smoke tests
- `.github/workflows/tests.yml` — visible GitHub Actions benchmark pipeline
- `extract_fafb_circuit.py` — reproducible FAFB subgraph extraction
- `out/` — compact real-connectome circuit + report
- `reports/` — compact result summaries and figures
- `artifacts/` — portable research snapshot
- `data/README.md` — upstream dataset notes; raw FAFB downloads are intentionally excluded from Git

## Provenance

The original 2D benchmark was run on Canavar and has now been transferred intact into the repository. The real FAFB circuit extraction was performed on Zaku from the downloaded v783 tables. CI reruns the simulator independently on GitHub-hosted Ubuntu/Python 3.12 so results can be compared against the original Canavar run.
