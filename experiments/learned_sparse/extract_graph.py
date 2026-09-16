from __future__ import annotations

import csv
import gzip
import json
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "data" / "fafb783"
OUT = Path(__file__).resolve().parent / "graph.json"

CORE_TYPES = {"LC4", "LPLC2", "DNp03"}
NT_SIGN = {
    "ACH": 1.0,
    "GABA": -1.0,
    "GLUT": -1.0,
    "DA": 0.5,
    "SER": 0.5,
    "OCT": 0.5,
}


def read_types():
    types = {}
    with gzip.open(DATA / "consolidated_cell_types.csv.gz", "rt", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            types[row["root_id"]] = row.get("primary_type", "") or ""
    return types


def read_sides():
    sides = {}
    with gzip.open(DATA / "classification.csv.gz", "rt", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            sides[row["root_id"]] = row.get("side", "") or ""
    return sides


def connection_rows():
    with gzip.open(DATA / "connections.csv.gz", "rt", encoding="utf-8", newline="") as f:
        yield from csv.DictReader(f)


def main():
    types = read_types()
    sides = read_sides()
    by_type = defaultdict(set)
    for rid, typ in types.items():
        if typ:
            by_type[typ].add(rid)

    loom = set(by_type["LC4"]) | set(by_type["LPLC2"])
    dn = set(by_type["DNp03"])
    reached_from_loom = set()
    projects_to_dn = set()
    direct_syn = 0
    direct_edges = set()

    for row in connection_rows():
        pre, post = row["pre_root_id"], row["post_root_id"]
        syn = int(row["syn_count"])
        if pre in loom:
            reached_from_loom.add(post)
        if post in dn:
            projects_to_dn.add(pre)
        if pre in loom and post in dn:
            direct_syn += syn
            direct_edges.add((pre, post))

    intermediates = (reached_from_loom & projects_to_dn) - loom - dn
    selected = loom | dn | intermediates
    print(f"loom={len(loom)} dn={len(dn)} intermediates={len(intermediates)} selected={len(selected)}")

    # Keep the same topology used in the v0.1 connectome controller:
    # looming outputs plus selected inputs into DNp03.
    edge_syn = defaultdict(int)
    edge_nt = defaultdict(Counter)
    edge_neuropil = defaultdict(Counter)
    for row in connection_rows():
        pre, post = row["pre_root_id"], row["post_root_id"]
        if pre not in selected or post not in selected:
            continue
        if pre not in loom and post not in dn:
            continue
        key = (pre, post)
        syn = int(row["syn_count"])
        edge_syn[key] += syn
        edge_nt[key][row.get("nt_type", "") or "UNKNOWN"] += syn
        edge_neuropil[key][row.get("neuropil", "") or "UNKNOWN"] += syn

    node_ids = sorted(selected, key=int)
    index = {rid: i for i, rid in enumerate(node_ids)}
    incoming = defaultdict(int)
    for (pre, post), syn in edge_syn.items():
        incoming[post] += syn

    nodes = []
    for rid in node_ids:
        nodes.append({
            "id": rid,
            "type": types.get(rid, ""),
            "side": sides.get(rid, ""),
            "is_input": rid in loom,
            "is_output": rid in dn,
        })

    edges = []
    for (pre, post), syn in sorted(edge_syn.items(), key=lambda kv: (index[kv[0][0]], index[kv[0][1]])):
        nt = edge_nt[(pre, post)].most_common(1)[0][0]
        neuropil = edge_neuropil[(pre, post)].most_common(1)[0][0]
        sign = NT_SIGN.get(nt, 0.0)
        norm = syn / max(incoming[post], 1)
        edges.append({
            "pre": index[pre], "post": index[post],
            "pre_id": pre, "post_id": post,
            "syn_count": syn, "nt": nt, "neuropil": neuropil,
            "sign": sign, "base_weight": sign * norm,
        })

    dn_stats = {}
    for rid in sorted(dn, key=int):
        inc_edges = [e for e in edges if e["post_id"] == rid]
        dn_stats[rid] = {
            "side": sides.get(rid, ""),
            "incoming_edges_selected": len(inc_edges),
            "incoming_synapses_selected": sum(e["syn_count"] for e in inc_edges),
            "loom_direct_edges": sum(1 for e in inc_edges if e["pre_id"] in loom),
            "loom_direct_synapses": sum(e["syn_count"] for e in inc_edges if e["pre_id"] in loom),
        }

    payload = {
        "source": "FlyWire Codex FAFB v783",
        "selection": "LC4 + LPLC2 + DNp03 + two-hop intermediates",
        "nodes": nodes,
        "edges": edges,
        "stats": {
            "LC4": len(by_type["LC4"]),
            "LPLC2": len(by_type["LPLC2"]),
            "DNp03": len(by_type["DNp03"]),
            "two_hop_intermediates": len(intermediates),
            "selected_nodes": len(nodes),
            "selected_edges": len(edges),
            "direct_loom_to_dnp03_edges": len(direct_edges),
            "direct_loom_to_dnp03_synapses": direct_syn,
            "dnp03": dn_stats,
        },
    }
    OUT.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload["stats"], indent=2))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
