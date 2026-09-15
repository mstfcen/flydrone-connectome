# Phase 4 — PX4 + Gazebo SITL

Phase 4 moves the project across an external simulator/flight-stack boundary using the official PX4 Gazebo environment.

## Validation status

| Capability | Status | Evidence |
|---|---|---|
| PX4 connection and estimator readiness | ✅ | MAVSDK SITL smoke test |
| Arm → takeoff → telemetry → land | ✅ | Relative altitude verified at ~1.52 m |
| `gz_x500_lidar_2d` sensor | ✅ | Live 1080-ray Gazebo scan |
| Runtime wall insertion | ✅ | Gazebo create service returns success |
| MAVLink `OBSTACLE_DISTANCE` bridge | ⚠️ | No stream observed in current image |
| Closed-loop wall avoidance | 🚧 | Experimental offboard integration not yet validated |

## Workflows

- `phase4_px4.yml` — validated PX4/Gazebo flight smoke test.
- `phase4_lidar.yml` — validated lidar topic/sensor discovery.
- `phase4_obstacle.yml` — manual MAVLink obstacle-bridge diagnostic.
- `phase4_final.yml` — manual closed-loop avoidance experiment; WIP and expected to fail until integration is fixed.

## Sensor boundary

The current `gz_x500_lidar_2d` model exposes a scan at:

```text
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
```

The observed scan contains 1080 beams over approximately 270° with a 0.1–30 m range. The current next milestone is a repeatable direct-scan → avoidance policy → MAVSDK Offboard → PX4 trajectory.