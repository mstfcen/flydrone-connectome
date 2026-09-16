import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EXP = ROOT / "experiments" / "learned_sparse"


def read_curve():
    with (EXP / "pruning_selection_curve.csv").open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return [(int(r["k"]), float(r["success_rate"])) for r in rows]


def read_random():
    with (EXP / "random24_baseline.csv").open(encoding="utf-8-sig") as f:
        return [float(r["success_rate"]) for r in csv.DictReader(f)]


def main():
    curve = read_curve()
    confirm = json.loads((EXP / "confirmatory60_summary.json").read_text())
    extra = json.loads((EXP / "confirmatory_extra.json").read_text())
    ks = np.array([k for k, _ in curve], dtype=float)
    success = np.array([100 * v for _, v in curve], dtype=float)
    fig, ax = plt.subplots(figsize=(8.6, 4.9))
    ax.plot(ks, success, marker="o", label="selection set")
    ax.scatter([377, 24],
               [100 * confirm["unpruned_377"]["success_rate"],
                100 * confirm["fixed_24"]["success_rate"]],
               marker="x", s=90, label="fixed-24 confirmation")
    ax.scatter([48, 40, 32],
               [100 * extra[str(k)]["success_rate"] for k in (48, 40, 32)],
               marker="s", s=45, label="post-hoc confirmation characterization")
    ax.set_xscale("log")
    ax.invert_xaxis()
    ax.set_ylim(0, 100)
    ax.set_xlabel("Active neurons (log scale; smaller →)")
    ax.set_ylabel("Closed-loop success (%)")
    ax.set_title("Learned connectome sparsification")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(EXP / "pruning_curve.png", dpi=180)
    plt.close(fig)
    random_vals = np.array(read_random()) * 100.0
    learned = 100.0 * json.loads((EXP / "selected24_summary.json").read_text())["learned_24"]["success_rate"]
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.bar([0, 1], [learned, random_vals.mean()], width=0.58)
    ax.scatter(np.full(len(random_vals), 1.0), random_vals, s=34, zorder=3)
    ax.set_xticks([0, 1], ["learned 24", "random 24\nmean"])
    ax.set_ylim(0, 100)
    ax.set_ylabel("Closed-loop success (%)")
    ax.set_title("Learned node ranking vs matched random masks")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(EXP / "selected24_vs_random.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
