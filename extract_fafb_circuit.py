import csv, gzip, json, math
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data" / "fafb783"
OUT = ROOT / "out"
OUT.mkdir(exist_ok=True)

TYPES = DATA / "consolidated_cell_types.csv.gz"
CLASS = DATA / "classification.csv.gz"
CONN = DATA / "connections.csv.gz"
CORE = {"LC4", "LPLC2", "DNp03"}
NT_SIGN = {"ACH": 1.0, "GABA": -1.0, "GLUT": -1.0,
           "DA": 0.5, "SER": 0.5, "OCT": 0.5}

type_of = {}
ids_by_type = defaultdict(set)
with gzip.open(TYPES, "rt", newline="") as f:
    rows = csv.reader(f); next(rows)
    for row in rows:
        rid, ctype = row[0], row[1].strip()
        type_of[rid] = ctype
        if ctype in CORE:
            ids_by_type[ctype].add(rid)

selected_for_meta = set().union(*ids_by_type.values())
meta = {}
with gzip.open(CLASS, "rt", newline="") as f:
    rows = csv.reader(f); next(rows)
    for row in rows:
        meta[row[0]] = {
            "flow": row[1], "super_class": row[2], "class": row[3],
            "sub_class": row[4], "side": row[6], "nerve": row[7]
        }

loom = ids_by_type["LC4"] | ids_by_type["LPLC2"]
dn = ids_by_type["DNp03"]
loom_out = Counter()
dn_in = Counter()
direct = Counter()
with gzip.open(CONN, "rt", newline="") as f:
    rows = csv.reader(f); next(rows)
    for pre, post, neuropil, syn, nt in rows:
        w = int(syn)
        if pre in loom:
            loom_out[post] += w
            if post in dn:
                direct[(pre, post)] += w
        if post in dn:
            dn_in[pre] += w

mid_ids = (set(loom_out) & set(dn_in)) - loom - dn
ranked_mid = sorted(
    mid_ids,
    key=lambda rid: math.sqrt(loom_out[rid] * dn_in[rid]),
    reverse=True,
)
selected = loom | dn | set(ranked_mid)

pair_syn = Counter()
pair_nt = defaultdict(Counter)
pair_neuropil = defaultdict(Counter)
with gzip.open(CONN, "rt", newline="") as f:
    rows = csv.reader(f); next(rows)
    for pre, post, neuropil, syn, nt in rows:
        if pre not in selected or post not in selected:
            continue
        if not (pre in loom or post in dn):
            continue
        key = (pre, post)
        w = int(syn)
        pair_syn[key] += w
        pair_nt[key][nt] += w
        pair_neuropil[key][neuropil] += w

incoming = Counter()
for (pre, post), syn in pair_syn.items():
    incoming[post] += syn

nodes = []
for rid in sorted(selected):
    role = type_of.get(rid, "")
    if rid in ranked_mid:
        role = role or "intermediate"
    nodes.append({"id": rid, "type": type_of.get(rid, ""),
                  "role": role, **meta.get(rid, {})})

edges = []
for (pre, post), syn in pair_syn.items():
    nt = pair_nt[(pre, post)].most_common(1)[0][0]
    sign = NT_SIGN.get(nt, 0.0)
    edges.append({
        "pre": pre, "post": post, "syn": syn, "nt": nt,
        "sign": sign, "weight": sign * syn / max(incoming[post], 1),
        "neuropils": dict(pair_neuropil[(pre, post)]),
    })

report = {
    "source": "FlyWire Codex FAFB v783",
    "core_counts": {k: len(v) for k, v in ids_by_type.items()},
    "direct_loom_to_dnp03_synapses": sum(direct.values()),
    "direct_loom_to_dnp03_edges": len(direct),
    "two_hop_intermediates": len(ranked_mid),
    "selected_nodes": len(nodes), "selected_edges": len(edges),
    "top_intermediates": [
        {"id": rid, "type": type_of.get(rid, ""),
         "loom_syn": loom_out[rid], "dnp03_syn": dn_in[rid],
         "score": math.sqrt(loom_out[rid] * dn_in[rid])}
        for rid in ranked_mid[:20]
    ],
}

(OUT / "fafb_loom_dnp03_circuit.json").write_text(
    json.dumps({"nodes": nodes, "edges": edges, "report": report}, indent=2)
)
(OUT / "fafb_loom_dnp03_report.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
