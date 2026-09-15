# FlyDrone

### Connectome-Inspired Reactive Obstacle Avoidance for Micro-UAVs

[![Core CI](https://github.com/mstfcen/flydrone-connectome/actions/workflows/tests.yml/badge.svg)](https://github.com/mstfcen/flydrone-connectome/actions/workflows/tests.yml)
[![PX4 SITL](https://github.com/mstfcen/flydrone-connectome/actions/workflows/phase4_px4.yml/badge.svg)](https://github.com/mstfcen/flydrone-connectome/actions/workflows/phase4_px4.yml)
[![Lidar Discovery](https://github.com/mstfcen/flydrone-connectome/actions/workflows/phase4_lidar.yml/badge.svg)](https://github.com/mstfcen/flydrone-connectome/actions/workflows/phase4_lidar.yml)

FlyDrone is a research prototype investigating whether compact **Drosophila-inspired visuomotor motifs**, informed by the FlyWire connectome, can provide robust reactive obstacle avoidance for micro-UAVs.

The project deliberately progresses through increasingly realistic validation layers: deterministic 2D arenas, idealized 3D motion, a four-motor 6-DoF rigid-body quadcopter, and finally an external **PX4 + Gazebo SITL** boundary.

> **Research question:** can a small temporal threat-estimation controller retain useful avoidance behavior when sensing is noisy, control is delayed, and the vehicle is disturbed by wind?

![6-DoF robustness benchmark](prototype/3d/stress_success_6dof.png)

## Headline result

In the current 6-DoF dense-scene stress suite (10 held-out runs per profile), the modified fly-inspired controller retained 100% success across all tested perturbations.
| Stress profile | Modified fly | Bilateral FlyWire | Classical VFH |
|---|---:|---:|---:|
| Clean | **100%** | 100% | 100% |
| Sensor noise + dropout | **100%** | 100% | 90% |
| 160 ms command latency | **100%** | 100% | 100% |
| Gusty wind | **100%** | 70% | 70% |
| Combined | **100%** | 90% | 60% |

These results are **simulation evidence, not a claim of general biological superiority**. The sample sizes are intentionally modest, the environments are synthetic, and the bilateral FlyWire controller includes an explicit engineering intervention. See [Limitations](docs/LIMITATIONS.md).

## What is connectome-derived?

A compact circuit is extracted reproducibly from **FlyWire FAFB v783** around looming-sensitive visual pathways and the DNp03 descending-neuron target:

- 104 LC4 neurons
- 210 LPLC2 neurons
- 2 DNp03 neurons
- 317 direct LC4/LPLC2 → DNp03 synapses across 37 directed edges
- 61 two-hop intermediate neurons
- 377 selected neurons and 1,117 selected edges in the exported circuit

The raw graph and the engineered bilateral variant are kept separate throughout the benchmark.
## System overview

```mermaid
flowchart LR
    FW[FlyWire FAFB v783] --> EX[Subgraph extraction]
    EX --> RAW[Raw FlyWire policy]
    EX --> BI[Engineered bilateral policy]
    S[Range / looming input] --> MF[Modified fly policy]
    S --> VFH[Classical VFH baseline]
    RAW --> EVAL[Controller evaluation]
    BI --> EVAL
    MF --> EVAL
    VFH --> EVAL
    EVAL --> D2[2D]
    D2 --> D3[3D]
    D3 --> RB[6-DoF rigid body]
    RB --> PX4[PX4 + Gazebo SITL]
```

The low-level vehicle dynamics are shared between policies so the staged comparisons isolate the avoidance layer as much as possible. Architecture details are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Validation ladder

| Layer | Status | What is validated |
|---|---|---|
| 2D arena | ✅ | Held-out nominal + noise/latency benchmarks |
| Idealized 3D | ✅ | 105-ray body-mounted retina, 3D avoidance |
| 6-DoF quadcopter | ✅ | Four motors, attitude/rates, drag, motor lag, wind |
| PX4/Gazebo flight boundary | ✅ | Connect, arm, take off, telemetry, land |
| Gazebo `x500_lidar_2d` | ✅ | Live 1080-ray scan, ~270° FOV, 0.1–30 m |
| Runtime obstacle spawn | ✅ | Physical wall inserted into Gazebo world |
| MAVLink obstacle bridge | ⚠️ | No automatic `OBSTACLE_DISTANCE` stream observed |
| Closed-loop PX4 avoidance | 🚧 | Experimental; current offboard integration is not validated |
## Controllers under test

**Modified fly** — a compact looming/proximity controller with time-to-collision estimation, temporal memory, braking and directional escape selection.

**Raw FlyWire** — a controller driven directly by the extracted FAFB connectivity. It is intentionally retained even though it fails the current 3D/6-DoF tasks; this is an important negative result.

**Bilateral FlyWire** — an explicit engineering variant that mirrors the well-connected hemisphere to provide a balanced left/right action pathway. It must not be interpreted as an untouched biological reconstruction.

**Classical VFH** — a vector-field-histogram-style reactive baseline optimized for goal alignment and obstacle clearance.

## Staged results

The progression matters more than any single percentage. A policy that looks strong in a simple geometric arena may fail after vehicle dynamics, latency or disturbances are introduced.

![6-DoF controller trajectories](prototype/3d/demo_trajectories_6dof.png)

![6-DoF nominal benchmark](prototype/3d/success_rates_6dof.png)

Detailed 2D, 3D and 6-DoF protocols and tables are in [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md). Raw episode tables and JSON summaries remain committed alongside the benchmark code.
## Reproduce the simulation benchmarks

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
pytest -q
python prototype/3d/benchmark6dof.py
python prototype/3d/stress6dof.py
```

GitHub Actions independently reruns the staged simulation suite on Ubuntu/Python 3.12. PX4/Gazebo boundary checks use the official `px4io/px4-sitl-gazebo` container.

## Repository layout

```text
extract_fafb_circuit.py   reproducible connectome subgraph extraction
out/                      compact extracted circuit + report
prototype/2d/             2D baseline and robustness experiments
prototype/3d/             3D + 6-DoF simulations and results
phase4/                   PX4/Gazebo SITL boundary experiments
tests/                    deterministic regression/smoke tests
docs/                     architecture, experiments and limitations
.github/workflows/         reproducible CI and SITL diagnostics
```

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Experiment design and results](docs/EXPERIMENTS.md)
- [Limitations and open questions](docs/LIMITATIONS.md)
- [PX4/Gazebo integration status](phase4/README.md)

FlyDrone is intentionally scoped as a compact research prototype. The next meaningful milestone is a validated closed-loop PX4/Gazebo obstacle-avoidance trial, followed by hardware-in-the-loop or controlled real-vehicle testing.