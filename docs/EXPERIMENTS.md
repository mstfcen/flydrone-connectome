# Experiments

This document records the staged benchmark design and the checked-in results. Percentages below are outcomes from deterministic synthetic simulation unless a PX4/Gazebo boundary test is explicitly named.

## Phase 1 — 2D

The first benchmark uses 90 held-out maps: 30 sparse, 30 cluttered and 30 dense. All controllers receive the same forward range observations and share the same kinematic limits.

| Controller | Sparse | Cluttered | Dense | Overall |
|---|---:|---:|---:|---:|
| Fly reflex | 63.3% | 86.7% | 80.0% | 76.7% |
| Modified fly | 76.7% | 93.3% | **100%** | 90.0% |
| Classical VFH | **100%** | **100%** | **100%** | **100%** |

A dense-scene stress suite then adds sensor noise/dropout and 160 ms high-level control latency.

| Profile | Fly reflex | Modified fly | VFH |
|---|---:|---:|---:|
| Clean | 76.7% | 100% | 100% |
| Sensor noise + 3% dropout | 90.0% | 86.7% | 46.7% |
| 160 ms delay | 66.7% | 90.0% | 100% |
| Combined | 86.7% | **93.3%** | 50.0% |

The counter-intuitive improvement of the simple fly reflex under noise is one reason the project treats these results as hypothesis-generating rather than conclusive.
## Phase 2 — idealized 3D

The 3D environment uses position/velocity dynamics and a 105-ray retina (21 azimuth × 5 elevation). Sixteen held-out maps are evaluated per difficulty.

| Controller | Sparse | Cluttered | Dense |
|---|---:|---:|---:|
| Heuristic fly | 68.8% | 81.2% | 50.0% |
| Modified fly | 87.5% | **100%** | **100%** |
| Raw FlyWire | 0% | 0% | 0% |
| Bilateral FlyWire | **100%** | **100%** | **100%** |
| Classical VFH | **100%** | 93.8% | **100%** |

The raw FlyWire policy collides in every 3D held-out run. Inspection of the extracted circuit showed a strongly asymmetric DNp03 input distribution; the bilateral variant is therefore reported separately rather than silently correcting the raw result.

## Phase 3 — 6-DoF rigid-body quadcopter

Phase 3 adds four motors, gravity, roll/pitch/yaw attitude and rates, inertia, thrust mixing, motor lag, aerodynamic drag, saturation and deterministic wind/gust profiles. Twelve held-out maps are evaluated per difficulty.

| Controller | Sparse | Cluttered | Dense |
|---|---:|---:|---:|
| Heuristic fly | **100%** | **100%** | **100%** |
| Modified fly | **100%** | **100%** | 91.7% |
| Raw FlyWire | 0% | 0% | 0% |
| Bilateral FlyWire | **100%** | 91.7% | **100%** |
| Classical VFH | **100%** | 91.7% | 91.7% |
The dense stress suite uses 10 runs per profile:

| Profile | Modified fly | Bilateral FlyWire | VFH |
|---|---:|---:|---:|
| Clean | **100%** | 100% | 100% |
| Sensor noise + dropout | **100%** | 100% | 90% |
| 160 ms latency | **100%** | 100% | 100% |
| Gusty wind | **100%** | 70% | 70% |
| Combined | **100%** | 90% | 60% |

The modified controller also uses comparatively little motor saturation in these runs; episode-level motor-effort, tilt and saturation metrics are available in `prototype/3d/results6dof.csv` and the JSON summaries.

## Phase 4 — PX4 + Gazebo boundary

The validated external-stack milestone uses the official `px4io/px4-sitl-gazebo` image. MAVSDK connects to PX4, waits for estimator/home readiness, arms the vehicle, commands takeoff, verifies approximately 1.52 m relative altitude, then lands and confirms the vehicle is no longer airborne.

The `gz_x500_lidar_2d` model exposes a live Gazebo scan with 1080 beams, approximately 270° field of view and a 0.1–30 m range. Runtime insertion of the test wall is also validated.

The closed-loop obstacle-avoidance integration is **not yet validated**. The current diagnostic found no automatic MAVLink `OBSTACLE_DISTANCE` bridge, and the experimental offboard wall trial timed out before reaching a valid avoidance trajectory. Those workflows are retained as diagnostics, not positive benchmark results.
## Phase 5 — learned sparse connectome (v0.2)

The v0.2 extension keeps the 377-node / 1,117-edge FlyWire-derived topology fixed while learning edge gains, node dynamics and node gates from `modified_fly` imitation data. Training uses 21,060 frames from 72 episodes, 6,068 validation frames from 24 episodes, 5% node dropout, 45 dense-imitation epochs and 110 sparsification epochs.

On a 60-map selection set, the smallest tested mask matching the unpruned 83.3% success point estimate used 24 active neurons. Five matched random 24-node masks averaged 14.3% success.

A separate 60-map confirmation set was then opened after `k=24` was fixed:

| Policy | Success | Collision |
|---|---:|---:|
| Modified-fly teacher | 93.3% | 6.7% |
| Learned 377-node | **85.0%** | 10.0% |
| Fixed learned 24-node | 76.7% | 21.7% |

The aggressive 24-node model therefore retained most but not all independent-set performance. Post-hoc characterization of the previously defined 48/40/32 masks yielded 83.3%, 86.7% and 85.0% success on the same confirmation maps, suggesting a more conservative sparse regime around 32–40 nodes. See `docs/LEARNED_SPARSE_REPORT.md` for the full protocol and interpretation.
