import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXP = ROOT / "experiments" / "learned_sparse"


def load(name):
    return json.loads((EXP / name).read_text(encoding="utf-8"))


def test_sparse_selection_summary_contract():
    s = load("selected24_summary.json")
    assert s["original_nodes"] == 377
    assert s["original_edges"] == 1117
    assert s["selected_nodes"] == 24
    assert 15.0 < s["compression_ratio"] < 16.0
    assert abs(s["unpruned_learned"]["success_rate"] - 5 / 6) < 1e-9
    assert abs(s["learned_24"]["success_rate"] - 5 / 6) < 1e-9
    assert s["random_24"]["mean_success_rate"] < 0.20


def test_independent_confirmation_is_kept_separate():
    c = load("confirmatory60_summary.json")
    assert c["protocol"] == "confirmatory after fixing k=24"
    assert c["unpruned_377"]["n"] == 60
    assert c["fixed_24"]["n"] == 60
    assert c["unpruned_377"]["success_rate"] == 0.85
    assert c["fixed_24"]["success_rate"] < c["unpruned_377"]["success_rate"]

def test_minimal_circuit_integrity():
    m = load("minimal_circuit24.json")
    assert len(m["nodes"]) == 24
    assert len(m["edges"]) == 25
    ids = {n["id"] for n in m["nodes"]}
    assert len(ids) == 24
    assert all(e["pre_id"] in ids and e["post_id"] in ids for e in m["edges"])
    assert sum(n["type"] == "DNp03" for n in m["nodes"]) == 1


def test_sparse_training_summary_contract():
    t = load("training_summary.json")
    assert t["nodes"] == 377
    assert t["edges"] == 1117
    assert t["gate_mean"] < 0.10
    assert t["gate_median"] < 0.01
    assert t["gate_lt_0.5"] >= 350
