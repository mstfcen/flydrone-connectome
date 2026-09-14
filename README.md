# FlyDrone Connectome Prototype

Bio-inspired UAV obstacle-avoidance research prototype comparing a fly-like looming reflex, a modified temporal-memory fly controller, and a classical VFH-style controller. The project has now begun replacing the hand-designed biological abstraction with a real FlyWire FAFB v783 subgraph.

## Current results

Nominal held-out benchmark: 90 maps total (30 sparse, 30 cluttered, 30 dense). Overall success was ~76.7% for the fly-reflex controller, 90.0% for modified-fly, and 100% for classical VFH. In dense scenes modified-fly reached 30/30.

Robustness test (dense maps, 30 runs/profile): with range noise + 3% ray dropout, success was 90.0% fly-reflex, 86.7% modified-fly, 46.7% VFH. With noise/dropout + 160 ms control delay, success was 86.7%, 93.3%, and 50.0% respectively.

These are prototype simulation results, not evidence that a biological controller is generally superior. The noise result is a hypothesis-generating observation and needs larger reruns and stronger baselines.

## Real connectome extraction

The Zaku mirror contains a compact circuit extracted from FlyWire FAFB v783: 104 LC4, 210 LPLC2 and 2 DNp03 neurons. In this snapshot the extraction found 317 direct LC4/LPLC2-to-DNp03 synapses over 37 directed edges, plus 61 two-hop intermediates. The exported compact circuit contains 377 selected neurons and 1,117 selected edges.

See `out/fafb_loom_dnp03_circuit.json` and `out/fafb_loom_dnp03_report.json`.

## Files

- `extract_fafb_circuit.py` — reproducible FAFB subgraph extraction
- `out/` — compact real-connectome circuit + report
- `reports/` — benchmark CSVs, learned modified-fly parameters, regenerated result figures
- `artifacts/` — portable ZIP snapshot
- `data/README.md` — source/dataset notes; raw FAFB downloads are intentionally excluded from Git because they are upstream inputs, not project outputs

## Provenance note

The original 2D benchmark was run on Canavar. Canavar went offline before cloud export, so the CSVs and plots in `reports/` were reconstructed exactly from the recorded run summaries. The real FAFB circuit extraction was performed on Zaku from the downloaded v783 tables. The original Canavar trajectory image and Canavar-only working files will be added when that machine is reachable again.
