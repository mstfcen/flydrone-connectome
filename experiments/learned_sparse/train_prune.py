from __future__ import annotations

import json
import random
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

from model import MAX_SPEED, MAX_TURN, SparseConnectome

HERE = Path(__file__).resolve().parent
DEVICE = torch.device("cpu")
SEED = 20260916


def seed_all(seed=SEED):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def load_split(name):
    d = np.load(HERE / f"{name}.npz")
    return {
        "sensory": torch.tensor(d["sensory"], dtype=torch.float32),
        "goal": torch.tensor(d["goal_error"], dtype=torch.float32),
        "action": torch.tensor(d["action"], dtype=torch.float32),
        "threat": torch.tensor(d["threat"], dtype=torch.float32),
    }


def task_loss(model, sensory, goal, action, threat, hard_mask=None):
    pred_speed, pred_omega, _, _ = model(sensory, goal, hard_mask=hard_mask)
    speed_t = action[:, 0]
    omega_t = action[:, 1]
    e_speed = ((pred_speed - speed_t) / MAX_SPEED) ** 2
    e_omega = ((pred_omega - omega_t) / MAX_TURN) ** 2
    weight = 1.0 + 6.0 * torch.clamp(threat, 0.0, 1.5)
    per = e_omega + 0.45 * e_speed
    return torch.sum(per * weight) / torch.sum(weight), e_speed.mean(), e_omega.mean()


@torch.no_grad()
def evaluate(model, split, hard_mask=None, batch_size=1024):
    model.eval()
    n = len(split["sensory"])
    totals = np.zeros(4, dtype=float)
    for i in range(0, n, batch_size):
        sl = slice(i, min(i + batch_size, n))
        loss, es, eo = task_loss(
            model,
            split["sensory"][sl].to(DEVICE),
            split["goal"][sl].to(DEVICE),
            split["action"][sl].to(DEVICE),
            split["threat"][sl].to(DEVICE),
            hard_mask=hard_mask,
        )
        b = len(split["sensory"][sl])
        totals += np.array([float(loss), float(es), float(eo), b]) * np.array([b, b, b, 1])
    return {
        "weighted_loss": totals[0] / totals[3],
        "speed_rmse_norm": float(np.sqrt(totals[1] / totals[3])),
        "omega_rmse_norm": float(np.sqrt(totals[2] / totals[3])),
    }


def train_stage(model, train, val, optimizer, epochs, sparsity_start, sparsity_end, stage_name):
    ds = TensorDataset(train["sensory"], train["goal"], train["action"], train["threat"])
    loader = DataLoader(ds, batch_size=512, shuffle=True, drop_last=False)
    history = []
    for epoch in range(1, epochs + 1):
        model.train()
        frac = 0.0 if epochs <= 1 else (epoch - 1) / (epochs - 1)
        lam = sparsity_start + frac * (sparsity_end - sparsity_start)
        running = 0.0
        seen = 0
        for sensory, goal, action, threat in loader:
            sensory = sensory.to(DEVICE)
            goal = goal.to(DEVICE)
            action = action.to(DEVICE)
            threat = threat.to(DEVICE)
            optimizer.zero_grad(set_to_none=True)
            task, _, _ = task_loss(model, sensory, goal, action, threat)
            p = model.gate_prob()
            entropy_push = torch.mean(p * (1.0 - p))
            loss = task + lam * model.sparsity_penalty() + 0.004 * entropy_push + 2e-4 * model.edge_regularizer()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            running += float(loss.detach()) * len(sensory)
            seen += len(sensory)
        if epoch == 1 or epoch % 10 == 0 or epoch == epochs:
            metrics = evaluate(model, val)
            gp = model.gate_prob().detach().cpu().numpy()
            rec = {
                "stage": stage_name,
                "epoch": epoch,
                "lambda": lam,
                "train_objective": running / seen,
                "val": metrics,
                "gate_mean": float(np.mean(gp)),
                "gate_lt_0.5": int(np.sum(gp < 0.5)),
                "gate_lt_0.2": int(np.sum(gp < 0.2)),
            }
            history.append(rec)
            print(json.dumps(rec))
    return history


def main():
    seed_all()
    torch.set_num_threads(max(1, min(8, torch.get_num_threads())))
    train = load_split("train")
    val = load_split("val")
    model = SparseConnectome().to(DEVICE)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2.5e-3, weight_decay=2e-5)

    history = []
    history += train_stage(model, train, val, optimizer, 45, 0.0, 0.0, "imitation")
    history += train_stage(model, train, val, optimizer, 110, 0.005, 0.085, "sparsify")

    final_val = evaluate(model, val)
    gate = model.gate_prob().detach().cpu().numpy()
    ranking = model.gate_ranking()
    checkpoint = {
        "model_state": model.state_dict(),
        "seed": SEED,
        "final_val": final_val,
    }
    torch.save(checkpoint, HERE / "checkpoint.pt")

    gate_rows = []
    for rank, (idx, prob, node) in enumerate(ranking, start=1):
        gate_rows.append({
            "rank": rank,
            "node_index": idx,
            "root_id": node["id"],
            "type": node.get("type", ""),
            "side": node.get("side", ""),
            "gate": prob,
            "is_input": bool(node.get("is_input")),
            "is_output": bool(node.get("is_output")),
        })

    import csv
    with (HERE / "gate_ranking.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(gate_rows[0]))
        w.writeheader()
        w.writerows(gate_rows)

    summary = {
        "seed": SEED,
        "nodes": model.n_nodes,
        "edges": int(len(model.pre)),
        "final_validation": final_val,
        "gate_mean": float(np.mean(gate)),
        "gate_median": float(np.median(gate)),
        "gate_lt_0.5": int(np.sum(gate < 0.5)),
        "gate_lt_0.2": int(np.sum(gate < 0.2)),
        "history": history,
        "top_20": gate_rows[:20],
        "bottom_20": gate_rows[-20:],
    }
    (HERE / "training_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "history" and k not in ("top_20", "bottom_20")}, indent=2))


if __name__ == "__main__":
    main()
