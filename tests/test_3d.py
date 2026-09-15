import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
P3 = ROOT / "prototype" / "3d"
sys.path.insert(0, str(P3))

from sim3d import make_world, raycast, MAX_RANGE
from controllers3d import (
    ModifiedFly3DController,
    BilateralFlyWire3DController,
    FlyWireGraph3DController,
)
from benchmark3d import run_episode


def test_3d_retina_contract():
    world = make_world(12000, "dense")
    ranges, az, el = raycast(world.start, world)
    assert len(ranges) == len(az) == len(el) == 105
    assert np.all(np.isfinite(ranges))
    assert np.all((ranges >= 0.0) & (ranges <= MAX_RANGE))


def test_modified_fly_known_dense_seed():
    r = run_episode(ModifiedFly3DController(), make_world(12000, "dense"))
    assert r["success"] and not r["collision"]

def test_bilateral_flywire_known_dense_seed():
    r = run_episode(BilateralFlyWire3DController(), make_world(12000, "dense"))
    assert r["success"] and not r["collision"]


def test_raw_graph_and_bilateral_are_distinct():
    world = make_world(12000, "dense")
    raw = run_episode(FlyWireGraph3DController(), world)
    mirrored = run_episode(BilateralFlyWire3DController(), world)
    assert raw["collision"]
    assert mirrored["success"]
