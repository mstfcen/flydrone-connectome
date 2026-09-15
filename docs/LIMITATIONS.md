# Limitations

FlyDrone is a research prototype. The repository is designed to make both positive and negative results visible rather than present a production-ready autonomy stack.

## Biological interpretation

The modified fly controller is **bio-inspired**, not a faithful whole-brain simulation. The FlyWire connectome provides structural connectivity, but a wiring diagram does not fully specify membrane dynamics, neuromodulation, learning state, electrical synapses or the animal's complete sensorimotor context.

The bilateral FlyWire controller is an engineering intervention. It mirrors the better-connected side of the selected DNp03 pathway to obtain balanced left/right control. Its performance must not be described as the performance of the untouched connectome.

The raw FlyWire policy failing in the current 3D and 6-DoF tasks is therefore an important result, not evidence that the biological circuit itself is ineffective.

## Experimental scope

The synthetic benchmark sets are modest in size and share common environment generators. Reported percentages should not be extrapolated to arbitrary environments.

Noise, latency and wind profiles are controlled stressors rather than a comprehensive model of real sensing and aerodynamics.

The classical baseline is VFH-style reactive avoidance, not an exhaustive comparison against modern mapping, planning, learned perception or model-predictive-control systems.

No statistical significance claim is made from the current sample sizes.
## PX4/Gazebo status

PX4 SITL takeoff/landing and the Gazebo lidar sensor boundary are validated independently. Closed-loop obstacle avoidance through PX4 Offboard is still work in progress.

A physical Gazebo wall can be spawned successfully, but the current PX4/Gazebo image did not automatically expose that lidar data as MAVLink `OBSTACLE_DISTANCE`. A later direct-Gazebo-scan offboard experiment also timed out before producing a valid avoidance trajectory.

For that reason, the public status table marks closed-loop PX4 avoidance as **experimental**, and the associated GitHub Actions workflows are diagnostics rather than success badges.

## Hardware

No real vehicle flight, hardware-in-the-loop validation, event camera, propeller/ESC characterization or safety certification is included.

A meaningful next step would be to validate the high-level controller against PX4/Gazebo with repeatable obstacle trials, then repeat the same protocol in hardware-in-the-loop before considering controlled physical flight.