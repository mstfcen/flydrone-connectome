import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
P3 = ROOT / "prototype" / "3d"
sys.path.insert(0, str(P3))

from sim3d import make_demo_world, make_world
from controllers3d import ModifiedFly3DController, BilateralFlyWire3DController
from rigidbody import (
    PHYS_DT, HOVER_THRUST, make_rigid_state, high_level_to_motor_targets,
    physics_step, raycast_rigid,
)
from benchmark6dof import run_episode


def test_6dof_hover_equilibrium():
    world = make_demo_world()
    state = make_rigid_state(world)
    for k in range(int(3.0 / PHYS_DT)):
        targets = high_level_to_motor_targets(state, (0.0, 0.0, 0.0))
        state, collision, _ = physics_step(state, targets, world, k * PHYS_DT)
        assert not collision
    assert np.linalg.norm(state[:3] - world.start[:3]) < 1e-6
    assert np.linalg.norm(state[3:6]) < 1e-6


def test_6dof_retina_moves_with_attitude():
    world = make_world(15000, "dense")
    state = make_rigid_state(world)
    r0, _, _ = raycast_rigid(state, world)
    state[7] = np.deg2rad(20.0)
    r1, _, _ = raycast_rigid(state, world)
    assert len(r0) == len(r1) == 105
    assert not np.allclose(r0, r1)


def test_modified_fly_known_6dof_seed():
    result = run_episode(ModifiedFly3DController(), make_world(15001, "dense"))
    assert result["success"] and not result["collision"]
    assert result["max_tilt_deg"] < 78.0


def test_bilateral_flywire_known_6dof_seed():
    result = run_episode(BilateralFlyWire3DController(), make_world(15000, "dense"))
    assert result["success"] and not result["collision"]
    assert result["motor_saturation_frac"] < 0.25
