from pathlib import Path
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]
EXP = ROOT / "experiments" / "learned_sparse"
sys.path.insert(0, str(EXP))

from model import MAX_SPEED, MAX_TURN, SparseConnectome

model = SparseConnectome()
checkpoint = torch.load(EXP / "checkpoint.pt", map_location="cpu", weights_only=True)
model.load_state_dict(checkpoint["model_state"])
model.eval()

assert model.n_nodes == 377
assert len(model.pre) == 1117
mask = model.topk_mask(24)
assert int(mask.sum().item()) == 24
sensory = torch.zeros((2, 31), dtype=torch.float32)
sensory[0, 15] = 1.0
sensory[1, 8:12] = 0.5
goal_error = torch.tensor([0.0, 0.25], dtype=torch.float32)

with torch.no_grad():
    speed, omega, left, right = model(sensory, goal_error, hard_mask=mask)

for tensor in (speed, omega, left, right):
    assert torch.isfinite(tensor).all()
assert torch.all(speed >= 0.0) and torch.all(speed <= MAX_SPEED)
assert torch.all(torch.abs(omega) <= MAX_TURN + 1e-6)

print(
    "learned-sparse smoke OK:",
    f"nodes={model.n_nodes}",
    f"edges={len(model.pre)}",
    f"active={int(mask.sum().item())}",
)
