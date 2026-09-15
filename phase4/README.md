# Phase 4 — PX4 + Gazebo SITL

This phase moves beyond the project's custom simulator and validates the stack against the real PX4 flight controller in Gazebo.

The first milestone is intentionally small and binary: boot the official PX4 Gazebo image with the `gz_x500` model, connect through MAVSDK on UDP 14540, arm, take off to 3 m, verify telemetry altitude, land, and verify the vehicle is no longer airborne.

## Why this matters

Phase 1–3 compare avoidance controllers in project-owned simulation code. Phase 4 introduces an external flight stack and simulator boundary so failures in PX4 arming, estimator readiness, motor control, MAVLink or Gazebo are visible instead of being abstracted away.

## CI

The dedicated workflow is `.github/workflows/phase4_px4.yml`. It uses the official `px4io/px4-sitl-gazebo:latest` image with `HEADLESS=1`, Linux host networking and the `gz_x500` model.

Artifacts include the MAVSDK mission result JSON, client log and complete PX4/Gazebo container log.

Next milestone: run the modified-fly controller in PX4 Offboard mode, then add Gazebo depth/LiDAR input and benchmark the same robustness scenarios against a classical controller.
