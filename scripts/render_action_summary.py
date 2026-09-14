import json
from pathlib import Path

p = Path(__file__).resolve().parents[1] / 'prototype' / '2d'
nom = json.loads((p / 'summary.json').read_text())['summary']
stress = json.loads((p / 'stress_summary.json').read_text())

print('## FlyDrone benchmark results')
print()
print('### Held-out nominal benchmark')
print('| Controller | Difficulty | Success | Collision | Mean sim time |')
print('|---|---:|---:|---:|---:|')
for c in ('fly_connectome', 'modified_fly', 'classical_vfh'):
    for d in ('sparse', 'cluttered', 'dense'):
        x = nom[c][d]
        print(f"| `{c}` | {d} | {100*x['success_rate']:.1f}% | {100*x['collision_rate']:.1f}% | {x['mean_time_s']:.2f}s |")

print()
print('### Dense-map robustness')
print('| Profile | Fly | Modified fly | Classical VFH |')
print('|---|---:|---:|---:|')
for profile in ('clean', 'sensor_noise', 'delay_160ms', 'combined'):
    vals = [100*stress[profile][c]['success_rate'] for c in ('fly_connectome','modified_fly','classical_vfh')]
    print(f'| {profile} | {vals[0]:.1f}% | {vals[1]:.1f}% | {vals[2]:.1f}% |')

print()
print('Raw CSV/JSON, console logs, and all PNG figures are attached to this workflow run as an artifact.')
