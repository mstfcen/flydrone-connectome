from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PROTO2D = ROOT / "prototype" / "2d"
sys.path.insert(0, str(PROTO2D))
from controllers import ModifiedFlyController, goal_error
from sim import DT, MAX_SPEED, make_world, raycast, step

HERE = Path(__file__).resolve().parent
MAX_STEPS = 520


def retinal_features(ranges, prev_ranges, angles, params):
    if prev_ranges is None:
        closing = np.zeros_like(ranges)
    else:
        closing = np.maximum(prev_ranges - ranges, 0.0) / DT
    ttc = np.where(closing > 0.03, ranges / (closing + 1e-9), 99.0)
    ttc_risk = np.exp(-ttc / max(params["ttc_gain"], 0.2))
    proximity = np.clip((5.5 - ranges) / 5.5, 0.0, 1.0) ** 2
    front_w = np.clip(np.cos(angles), 0.0, 1.0) ** 2
    retinal = (ttc_risk + params["prox_gain"] * proximity) * (0.25 + 0.75 * front_w)
    return np.clip(retinal, 0.0, 1.5).astype(np.float32)


def load_teacher_params():
    payload = json.loads((PROTO2D / "summary.json").read_text(encoding="utf-8"))
    return payload["modified_fly_params"]


def collect(seeds, params):
    sensory, goals, actions, threats, episode_ids = [], [], [], [], []
    outcome = []
    for episode_id, seed in enumerate(seeds):
        difficulty = ("sparse", "cluttered", "dense")[episode_id % 3]
        world = make_world(seed, difficulty)
        teacher = ModifiedFlyController(params)
        teacher.reset()
        state = world.start.copy()
        prev_ranges = None
        success = collision = False
        for _ in range(MAX_STEPS):
            ranges, angles = raycast(state, world)
            retinal = retinal_features(ranges, prev_ranges, angles, params)
            prev_ranges = ranges.copy()
            ge = goal_error(state, world.goal)
            speed, omega = teacher.act(state, ranges, angles, world.goal)
            sensory.append(retinal)
            goals.append(ge)
            actions.append((speed, omega))
            threats.append(float(np.max(retinal)))
            episode_ids.append(episode_id)
            state, collision, success = step(state, speed, omega, world)
            if success or collision:
                break
        outcome.append((seed, difficulty, bool(success), bool(collision)))
    return {
        "sensory": np.asarray(sensory, dtype=np.float32),
        "goal_error": np.asarray(goals, dtype=np.float32),
        "action": np.asarray(actions, dtype=np.float32),
        "threat": np.asarray(threats, dtype=np.float32),
        "episode_id": np.asarray(episode_ids, dtype=np.int32),
        "outcome": outcome,
    }


def save_split(name, data):
    np.savez_compressed(
        HERE / f"{name}.npz",
        sensory=data["sensory"],
        goal_error=data["goal_error"],
        action=data["action"],
        threat=data["threat"],
        episode_id=data["episode_id"],
    )
    succ = sum(x[2] for x in data["outcome"])
    coll = sum(x[3] for x in data["outcome"])
    print(f"{name}: samples={len(data['sensory'])} episodes={len(data['outcome'])} success={succ} collision={coll}")


def main():
    params = load_teacher_params()
    train = collect(range(20000, 20072), params)
    val = collect(range(22000, 22024), params)
    save_split("train", train)
    save_split("val", val)
    meta = {
        "teacher": "modified_fly",
        "teacher_params": params,
        "train_seeds": [20000, 20071],
        "validation_seeds": [22000, 22023],
        "train_samples": len(train["sensory"]),
        "validation_samples": len(val["sensory"]),
        "input": "31-ray TTC/proximity retinal threat + goal error",
    }
    (HERE / "dataset_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
