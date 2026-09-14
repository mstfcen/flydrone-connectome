from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PROTO = ROOT / "prototype" / "2d"
sys.path.insert(0, str(PROTO))

from sim import MAX_RANGE, MAX_SPEED, MAX_TURN, make_world, raycast  # noqa: E402
from benchmark import run_episode  # noqa: E402
from controllers import (  # noqa: E402
    FlyConnectomeController,
    ModifiedFlyController,
    VFHController,
)


def test_world_generation_is_deterministic():
    a = make_world(7000, "dense")
    b = make_world(7000, "dense")
    assert np.allclose(a.obstacles, b.obstacles)
    assert np.allclose(a.start, b.start)
    assert np.allclose(a.goal, b.goal)


def test_raycast_contract():
    world = make_world(7000, "cluttered")
    ranges, angles = raycast(world.start, world)
    assert ranges.shape == angles.shape == (31,)
    assert np.isfinite(ranges).all()
    assert np.all((ranges >= 0.0) & (ranges <= MAX_RANGE))


@pytest.mark.parametrize(
    "factory",
    [FlyConnectomeController, ModifiedFlyController, VFHController],
)
def test_controller_action_contract(factory):
    world = make_world(7000, "dense")
    c = factory()
    c.reset()
    ranges, angles = raycast(world.start, world)
    speed, omega = c.act(world.start.copy(), ranges, angles, world.goal)
    assert np.isfinite([speed, omega]).all()
    assert 0.0 <= speed <= MAX_SPEED
    assert -MAX_TURN <= omega <= MAX_TURN


def test_known_dense_seed_modified_fly_reaches_goal():
    result = run_episode(ModifiedFlyController(), make_world(7000, "dense"))
    assert result["success"]
    assert not result["collision"]
