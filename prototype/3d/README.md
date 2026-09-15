# Phase 2: 3D micro-UAV benchmark

This phase moves FlyDrone out of the original planar benchmark. The state now contains 3D position and velocity plus yaw, with acceleration and velocity limits. Each controller receives the same 105-ray retina: 21 azimuth rays across 160 degrees by 5 elevation bands from -40 to +40 degrees.

This is an **inertial 3D micro-UAV model**, not yet a full quadcopter rigid-body / motor / propeller simulator. Attitude stabilization is implicit. The next fidelity step should therefore be PyBullet, Gazebo or AirSim/PX4 rather than claiming these numbers as hardware-flight results.

## Controllers

- `fly_heuristic_3d` — looming/proximity reflex extended into yaw + vertical avoidance.
- `modified_fly_3d` — TTC, temporal memory and 3D escape selection.
- `flywire_graph_3d` — raw 377-node / 1,117-edge FAFB v783 subgraph. Real topology and signed synaptic weights; synthetic retinotopic projection is required because the compact export does not contain receptive-field coordinates.
- `flywire_bilateral_3d` — engineering variant that evaluates the better-connected left DNp03 circuit on original and azimuth-mirrored retinal input. This is deliberately separate from the raw-connectome result.
- `classical_vfh_3d` — 3D VFH-style clearance/goal direction baseline.

## Important connectome finding

The extracted FAFB snapshot is strongly asymmetric at DNp03: the selected/raw graph has substantial input to left DNp03 and effectively no selected input to right DNp03. In the full upstream FAFB connection table, left DNp03 has 4,532 incoming synapses versus 140 for right DNp03. Raw graph control therefore saturates one side and fails; the bilateralized variant tests the engineering hypothesis that a mirrored homologous circuit can restore usable control.

## First held-out result (16 maps per difficulty)

| Controller | Sparse | Cluttered | Dense |
|---|---:|---:|---:|
| Heuristic fly | 68.8% | 81.2% | 50.0% |
| Modified fly | 87.5% | 100.0% | 100.0% |
| Raw FlyWire graph | 0.0% | 0.0% | 0.0% |
| Bilateralized FlyWire | 100.0% | 100.0% | 100.0% |
| Classical VFH | 100.0% | 93.8% | 100.0% |

In the separate 16-map dense robustness set, modified-fly and bilateralized FlyWire scored 100% under clean, range-noise/dropout, 160 ms delay, and the combined profile. Classical VFH ranged from 81.2% to 93.8%. These are early simulation results and should be rerun at larger N with wind, moving obstacles and rigid-body dynamics.

Generated files: `results3d.csv`, `summary3d.json`, `stress_summary3d.json`, `success_rates_3d.png`, `demo_trajectories_3d.png`, and `stress_success_3d.png`.