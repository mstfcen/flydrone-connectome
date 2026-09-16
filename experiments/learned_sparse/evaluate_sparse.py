from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PROTO2D = REPO / "prototype" / "2d"
sys.path[:0] = [str(PROTO2D), str(HERE)]

from controllers import ModifiedFlyController, goal_error
from sim import DT, make_world, raycast, step
from model import SparseConnectome

MAX_STEPS = 650
KS = [377, 192, 128, 96, 64, 48, 40, 32, 24, 20, 16, 12, 8]
SELECTION_SEEDS = list(range(25000, 25060))
RANDOM_SEEDS = list(range(27000, 27060))
ABLATION_SEEDS = list(range(26000, 26016))
CONFIRM_SEEDS = list(range(28000, 28060))
def wilson(successes: int, n: int, z: float = 1.959963984540054):
    p = successes / n
    den = 1.0 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return [mid - half, mid + half]


def load_model():
    model = SparseConnectome()
    ckpt = torch.load(HERE / "checkpoint.pt", map_location="cpu", weights_only=True)
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    return model


def teacher_params():
    payload = json.loads((PROTO2D / "summary.json").read_text(encoding="utf-8"))
    return payload["modified_fly_params"]


def retinal_features(ranges, prev_ranges, angles, params):
    closing = np.zeros_like(ranges) if prev_ranges is None else np.maximum(prev_ranges - ranges, 0.0) / DT
    ttc = np.where(closing > 0.03, ranges / (closing + 1e-9), 99.0)
    risk = np.exp(-ttc / max(params["ttc_gain"], 0.2))
    proximity = np.clip((5.5 - ranges) / 5.5, 0.0, 1.0) ** 2
    front_w = np.clip(np.cos(angles), 0.0, 1.0) ** 2
    return np.clip((risk + params["prox_gain"] * proximity) * (0.25 + 0.75 * front_w), 0.0, 1.5).astype(np.float32)
class LearnedPolicy:
    def __init__(self, model, mask, params):
        self.model = model
        self.mask = mask
        self.params = params
        self.reset()

    def reset(self):
        self.prev = None

    def act(self, state, ranges, angles, goal):
        sensory = retinal_features(ranges, self.prev, angles, self.params)
        self.prev = ranges.copy()
        ge = goal_error(state, goal)
        with torch.no_grad():
            s = torch.from_numpy(sensory).unsqueeze(0)
            g = torch.tensor([ge], dtype=torch.float32)
            speed, omega, _, _ = self.model(s, g, hard_mask=self.mask)
        return float(speed.item()), float(omega.item())


def run_episode(controller, world):
    controller.reset()
    state = world.start.copy()
    start_dist = float(np.linalg.norm(state[:2] - world.goal))
    success = collision = False
    min_clearance = 99.0
    for step_i in range(MAX_STEPS):
        ranges, angles = raycast(state, world)
        min_clearance = min(min_clearance, float(np.min(ranges)))
        speed, omega = controller.act(state, ranges, angles, world.goal)
        state, collision, success = step(state, speed, omega, world)
        if success or collision:
            break
    progress = 1.0 - float(np.linalg.norm(state[:2] - world.goal)) / start_dist
    return bool(success), bool(collision), step_i + 1, progress, min_clearance


def summarize_rows(rows):
    arr = np.asarray(rows, dtype=float)
    successes = int(arr[:, 0].sum())
    n = len(rows)
    return {
        "success_rate": successes / n,
        "collision_rate": float(arr[:, 1].mean()),
        "mean_steps": float(arr[:, 2].mean()),
        "mean_progress": float(arr[:, 3].mean()),
        "mean_min_clearance_m": float(arr[:, 4].mean()),
        "success_ci95": wilson(successes, n),
        "n": n,
    }


def eval_policy(factory, seeds):
    return summarize_rows([run_episode(factory(), make_world(seed, "dense")) for seed in seeds])


def eval_mask(model, mask, params, seeds):
    return eval_policy(lambda: LearnedPolicy(model, mask, params), seeds)


def random_mask(model, k, rng):
    out = int(model.output_idx)
    pool = np.asarray([i for i in range(model.n_nodes) if i != out], dtype=int)
    choose = rng.choice(pool, size=k - 1, replace=False)
    mask = torch.zeros(model.n_nodes, dtype=torch.float32)
    mask[torch.tensor(choose, dtype=torch.long)] = 1.0
    mask[out] = 1.0
    return mask
def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_curve(curve, confirm):
    ks = [r["k"] for r in curve]
    vals = [100 * r["success_rate"] for r in curve]
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    ax.plot(ks, vals, marker="o", label="selection set")
    ax.scatter([377, 24], [100 * confirm["unpruned_377"]["success_rate"],
                           100 * confirm["fixed_24"]["success_rate"]],
               marker="x", s=90, label="independent confirmation")
    ax.set_xscale("log")
    ax.invert_xaxis()
    ax.set_xlabel("Active neurons (log scale; smaller →)")
    ax.set_ylabel("Closed-loop success (%)")
    ax.set_title("Connectome-constrained pruning curve")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(HERE / "pruning_curve.png", dpi=170)
    plt.close(fig)


def plot_random(selected, random_rows):
    vals = [100 * r["success_rate"] for r in random_rows]
    fig, ax = plt.subplots(figsize=(7.4, 4.5))
    ax.bar(["learned 24", "random 24\n(mean)"],
           [100 * selected["success_rate"], float(np.mean(vals))])
    ax.scatter(np.full(len(vals), 1.0), vals, zorder=3, label="random masks")
    ax.set_ylim(0, 100)
    ax.set_ylabel("Closed-loop success (%)")
    ax.set_title("Learned importance vs matched random pruning")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(HERE / "selected24_vs_random.png", dpi=170)
    plt.close(fig)
def main():
    torch.manual_seed(20260916)
    np.random.seed(20260916)
    torch.set_num_threads(min(8, max(1, torch.get_num_threads())))
    model = load_model()
    params = teacher_params()

    curve = []
    for k in KS:
        metrics = eval_mask(model, model.topk_mask(k), params, SELECTION_SEEDS)
        row = {"k": k, **metrics}
        curve.append(row)
        print("SELECTION", json.dumps(row), flush=True)

    baseline = curve[0]["success_rate"]
    viable = [r for r in curve if r["success_rate"] + 1e-12 >= baseline]
    selected_k = min(r["k"] for r in viable)
    selected_mask = model.topk_mask(selected_k)

    rng = np.random.default_rng(20260917)
    random_rows = []
    for rep in range(5):
        metrics = eval_mask(model, random_mask(model, selected_k, rng), params, RANDOM_SEEDS)
        row = {"rep": rep, "k": selected_k, **metrics}
        random_rows.append(row)
        print("RANDOM", json.dumps(row), flush=True)
    base_ablation = eval_mask(model, selected_mask, params, ABLATION_SEEDS)
    gate = model.gate_prob().detach().cpu().numpy()
    ablations = []
    protected = {int(model.output_idx)}
    for idx in torch.where(selected_mask > 0.5)[0].tolist():
        node = model.nodes[idx]
        if idx in protected:
            ablations.append({"node_index": idx, "root_id": str(node["id"]),
                              "type": node.get("type", ""), "side": node.get("side", ""),
                              "gate": float(gate[idx]), "protected": True,
                              "base_success_rate": base_ablation["success_rate"],
                              "ablated_success_rate": "", "success_drop": ""})
            continue
        mask = selected_mask.clone()
        mask[idx] = 0.0
        metrics = eval_mask(model, mask, params, ABLATION_SEEDS)
        ablations.append({"node_index": idx, "root_id": str(node["id"]),
                          "type": node.get("type", ""), "side": node.get("side", ""),
                          "gate": float(gate[idx]), "protected": False,
                          "base_success_rate": base_ablation["success_rate"],
                          "ablated_success_rate": metrics["success_rate"],
                          "success_drop": base_ablation["success_rate"] - metrics["success_rate"]})

    confirm = {
        "protocol": "independent confirmation after fixing selected_k",
        "teacher": eval_policy(lambda: ModifiedFlyController(params), CONFIRM_SEEDS),
        "unpruned_377": eval_mask(model, model.topk_mask(377), params, CONFIRM_SEEDS),
        "fixed_selected": eval_mask(model, selected_mask, params, CONFIRM_SEEDS),
    }
    confirm_posthoc = {
        str(k): eval_mask(model, model.topk_mask(k), params, CONFIRM_SEEDS)
        for k in (48, 40, 32)
    }
    summary = {
        "experiment": "learned sparse connectome",
        "selection_rule": "smallest tested k matching unpruned selection-set success point estimate",
        "selection_set": [SELECTION_SEEDS[0], SELECTION_SEEDS[-1]],
        "confirmatory_set": [CONFIRM_SEEDS[0], CONFIRM_SEEDS[-1]],
        "selected_k": selected_k,
        "original_nodes": model.n_nodes,
        "original_edges": int(len(model.pre)),
        "compression_ratio": model.n_nodes / selected_k,
        "selection_curve": curve,
        "random_24": random_rows,
        "ablation": ablations,
        "confirmatory": confirm,
        "confirmatory_posthoc": confirm_posthoc,
    }
    (HERE / "evaluation_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (HERE / "confirmatory_extra.json").write_text(json.dumps(confirm_posthoc, indent=2), encoding="utf-8")
    write_csv(HERE / "pruning_curve.csv", curve)
    write_csv(HERE / "random24_baseline.csv", random_rows)
    write_csv(HERE / "ablation24.csv", ablations)
    plot_curve(curve, {"unpruned_377": confirm["unpruned_377"],
                       "fixed_24": confirm["fixed_selected"]})
    plot_random(curve[[r["k"] for r in curve].index(selected_k)], random_rows)
    print(json.dumps({"selected_k": selected_k,
                      "compression_ratio": model.n_nodes / selected_k,
                      "confirmatory": confirm}, indent=2))


if __name__ == "__main__":
    main()
