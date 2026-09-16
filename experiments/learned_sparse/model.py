from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import torch
from torch import nn

HERE = Path(__file__).resolve().parent
MAX_SPEED = 4.0
MAX_TURN = 3.8


def _atanh_clip(x):
    return np.arctanh(np.clip(x, -0.95, 0.95))


def build_ray_map(nodes):
    mapping = {}
    for typ in ("LC4", "LPLC2"):
        for side in ("left", "right", "", "center"):
            idxs = [i for i, n in enumerate(nodes) if n["type"] == typ and n.get("side", "") == side]
            if not idxs:
                continue
            idxs.sort(key=lambda i: int(nodes[i]["id"]))
            if side == "left":
                rays = np.arange(16, 31)
            elif side == "right":
                rays = np.arange(0, 15)
            else:
                rays = np.arange(31)
            pos = np.linspace(0, len(rays) - 1, len(idxs))
            for node_i, p in zip(idxs, pos):
                mapping[node_i] = int(rays[int(round(p))])
    return mapping


class SparseConnectome(nn.Module):
    """Trainable controller constrained to the extracted FlyWire graph topology."""

    def __init__(self, graph_path=HERE / "graph.json", node_dropout=0.05, propagation_steps=3):
        super().__init__()
        graph = json.loads(Path(graph_path).read_text(encoding="utf-8"))
        self.nodes = graph["nodes"]
        self.n_nodes = len(self.nodes)
        self.node_dropout = float(node_dropout)
        self.propagation_steps = int(propagation_steps)

        edges = graph["edges"]
        self.register_buffer("pre", torch.tensor([e["pre"] for e in edges], dtype=torch.long))
        self.register_buffer("post", torch.tensor([e["post"] for e in edges], dtype=torch.long))
        mag = np.asarray([abs(e["base_weight"]) for e in edges], dtype=np.float32)
        mag = np.maximum(mag, 1e-4)
        self.register_buffer("edge_mag", torch.tensor(mag, dtype=torch.float32))
        init_signed = np.asarray([np.sign(e["base_weight"]) for e in edges], dtype=np.float32)
        init_param = _atanh_clip(0.5 * init_signed).astype(np.float32)
        self.edge_param = nn.Parameter(torch.tensor(init_param))
        self.register_buffer("edge_init", torch.tensor(init_param))

        self.leak_logits = nn.Parameter(torch.full((self.n_nodes,), -0.4))
        self.gate_logits = nn.Parameter(torch.full((self.n_nodes,), 3.0))

        ray_map = build_ray_map(self.nodes)
        input_nodes = sorted(ray_map)
        self.register_buffer("input_nodes", torch.tensor(input_nodes, dtype=torch.long))
        self.register_buffer("input_rays", torch.tensor([ray_map[i] for i in input_nodes], dtype=torch.long))
        input_types = [self.nodes[i]["type"] for i in input_nodes]
        self.register_buffer("input_type_code", torch.tensor([0 if t == "LC4" else 1 for t in input_types], dtype=torch.long))
        self.input_type_gain = nn.Parameter(torch.tensor([1.0, 1.0], dtype=torch.float32))

        dn_left = [i for i, n in enumerate(self.nodes) if n["type"] == "DNp03" and n.get("side") == "left"]
        if len(dn_left) != 1:
            raise RuntimeError(f"expected one left DNp03, got {dn_left}")
        self.output_idx = dn_left[0]
        protected = torch.zeros(self.n_nodes, dtype=torch.bool)
        protected[self.output_idx] = True
        self.register_buffer("protected", protected)

        self.output_scale = nn.Parameter(torch.tensor(3.0))
        self.output_bias = nn.Parameter(torch.tensor(-0.5))
        self.goal_gain = nn.Parameter(torch.tensor(0.95))
        self.avoid_raw = nn.Parameter(torch.tensor(1.2))
        self.brake_raw = nn.Parameter(torch.tensor(0.6))

    def gate_prob(self):
        p = torch.sigmoid(self.gate_logits)
        return torch.where(self.protected, torch.ones_like(p), p)

    def effective_edge_weight(self):
        return self.edge_mag * 2.0 * torch.tanh(self.edge_param)

    def _node_gate(self, batch, hard_mask=None):
        gate = self.gate_prob()
        if hard_mask is not None:
            gate = gate * hard_mask.to(gate)
            gate = torch.where(self.protected, torch.ones_like(gate), gate)
        if self.training and self.node_dropout > 0:
            keep = (torch.rand((batch, self.n_nodes), device=gate.device) >= self.node_dropout).to(gate.dtype)
            keep[:, self.protected] = 1.0
            return gate.unsqueeze(0) * keep
        return gate.unsqueeze(0).expand(batch, -1)

    def hemisphere_score(self, sensory, hard_mask=None):
        batch = sensory.shape[0]
        gate = self._node_gate(batch, hard_mask)
        inp = torch.zeros((batch, self.n_nodes), dtype=sensory.dtype, device=sensory.device)
        drive = sensory[:, self.input_rays]
        type_gain = torch.nn.functional.softplus(self.input_type_gain)[self.input_type_code]
        inp[:, self.input_nodes] = drive * type_gain.unsqueeze(0)

        leak = torch.sigmoid(self.leak_logits).unsqueeze(0)
        h = gate * torch.tanh(inp)
        weights = self.effective_edge_weight()
        for _ in range(self.propagation_steps):
            msg = h[:, self.pre] * weights.unsqueeze(0)
            agg = torch.zeros_like(h)
            agg.index_add_(1, self.post, msg)
            h = gate * torch.tanh(leak * h + agg + 0.35 * inp)
        raw = self.output_scale * h[:, self.output_idx] + self.output_bias
        return torch.sigmoid(raw)

    def forward(self, sensory, goal_error, hard_mask=None):
        left_threat = self.hemisphere_score(sensory, hard_mask)
        right_threat = self.hemisphere_score(torch.flip(sensory, dims=[1]), hard_mask)
        avoid_gain = torch.nn.functional.softplus(self.avoid_raw)
        brake = torch.sigmoid(self.brake_raw)
        avoidance = avoid_gain * (right_threat - left_threat)
        raw_omega = self.goal_gain * goal_error + avoidance
        omega = MAX_TURN * torch.tanh(raw_omega / MAX_TURN)
        threat = torch.maximum(left_threat, right_threat)
        speed = MAX_SPEED * (1.0 - brake * threat)
        speed = torch.clamp(speed, min=0.9, max=MAX_SPEED)
        return speed, omega, left_threat, right_threat

    def topk_mask(self, k):
        k = int(max(1, min(k, self.n_nodes)))
        probs = self.gate_prob().detach().clone()
        probs[self.protected] = 2.0
        keep = torch.zeros_like(probs)
        idx = torch.topk(probs, k=k).indices
        keep[idx] = 1.0
        keep[self.protected] = 1.0
        return keep

    def sparsity_penalty(self):
        p = self.gate_prob()
        trainable = ~self.protected
        return p[trainable].mean()

    def edge_regularizer(self):
        return torch.mean((self.edge_param - self.edge_init) ** 2)

    def gate_ranking(self):
        p = self.gate_prob().detach().cpu().numpy()
        order = np.argsort(-p)
        return [(int(i), float(p[i]), self.nodes[int(i)]) for i in order]
