# Learned Sparse Connectome Extension — Final Report

## Objective

The v0.2 extension asks a narrower question than the original FlyDrone study:

> If the FlyWire-derived circuit topology is kept fixed, how much of the network can be suppressed while retaining useful closed-loop obstacle-avoidance behavior?

This is an exploratory engineering study, not a claim that the corresponding biological neurons are dispensable in a fly. The task, sensory projection, controller outputs and training objective are all UAV-specific.

## Starting circuit

The experiment uses the same FAFB v783-derived subgraph introduced in v0.1:

- 377 selected neurons
- 1,117 directed selected edges
- 104 LC4 neurons
- 210 LPLC2 neurons
- 2 DNp03 neurons
- 61 two-hop intermediate neurons

The extracted topology is fixed during learning. No new graph edges are created.

A known asymmetry remains in the selected snapshot: the left DNp03 has selected incoming connectivity while the right DNp03 has none. The learned controller therefore evaluates one shared learned circuit on normal and mirrored sensory input. This is an explicit engineering intervention, not a literal bilateral reconstruction.
## Learning setup

The previously tuned `modified_fly` controller acts as a teacher. Rollouts generated:

- 21,060 training frames from 72 episodes
- 6,068 validation frames from 24 episodes
- a 31-ray TTC/proximity retinal representation plus goal-heading error

The student learns only parameters inside the fixed connectome-constrained architecture:

- per-edge gain parameters
- per-node leak/dynamics parameters
- differentiable per-node gates
- a small set of output/control gains

Training uses 5% random node dropout. The first 45 epochs focus on imitation; the following 110 epochs gradually increase the sparsity penalty.

At the end of training the validation weighted loss was 0.0413. Mean gate probability fell to 0.074 and median gate probability to 0.0046; 357/377 nodes had gate probability below 0.5. Soft gates alone are not taken as evidence of dispensability, so the final conclusions use hard-pruned closed-loop evaluation.

## Evaluation design

Four distinct evaluation roles are kept separate:

1. **Selection set:** 60 dense maps used to trace the hard-pruning curve and choose an aggressive candidate size.
2. **Random-mask set:** 60 dense maps per mask, five matched 24-node random masks, without retraining.
3. **Ablation set:** 16 dense maps used to remove one retained node at a time from the 24-node exploratory circuit.
4. **Confirmation set:** 60 new dense maps opened only after `k=24` had been fixed.
## Results

### 1. Selection-set pruning curve

On the 60-map selection set, the unpruned 377-node learned controller achieved 83.3% success. The 24-node top-gate circuit also achieved 83.3%, giving a nominal 15.7× node-count compression on that set. Performance degraded at 20 nodes (70.0%) and collapsed at 8 nodes (11.7%).

The selection curve was not monotonic: 40 nodes reached 86.7% and 32 nodes reached 85.0%. These differences are small relative to the sample size and should not be interpreted as evidence that pruning intrinsically improves performance.

### 2. Matched random pruning

Five random 24-node masks achieved 33.3%, 18.3%, 6.7%, 6.7% and 6.7% success, for a mean of 14.3% across five masks. The learned 24-node selection therefore preserved substantially more task behavior than same-size random deletion under the same trained weights.

This does **not** compare against randomly wired networks retrained from scratch. It isolates whether the learned node ranking contains useful pruning information.

### 3. Independent fixed-24 confirmation

The independent 60-map confirmation set changed the strongest interpretation of the exploratory result:

| Policy | Success | Collision | Wilson 95% success interval |
|---|---:|---:|---:|
| Modified-fly teacher | 93.3% | 6.7% | 84.1–97.4% |
| Learned 377-node | **85.0%** | 10.0% | 73.9–91.9% |
| Fixed learned 24-node | **76.7%** | 21.7% | 64.6–85.6% |

Thus, the 24-node circuit preserved **most**, but not all, of the unpruned model's independent-set performance. The point-estimate difference is 8.3 percentage points, and the intervals overlap; this experiment is not designed to prove equivalence or significance.
### 4. Conservative sparse regime

After the fixed-24 confirmation was inspected, the already-defined 48/40/32 masks were characterized on the same confirmation maps. They achieved 83.3%, 86.7% and 85.0% success respectively, versus 85.0% for the unpruned model.

This suggests that a **roughly 32–40 neuron regime** is a more conservative engineering summary of the useful sparse core. Because these sizes were examined after the confirmation set had been opened, they are explicitly labeled post-hoc rather than confirmatory.

### 5. Single-node ablation

The exploratory 24-node motif contains 10 LC4, 6 LPLC4, 4 LPLC2, one PVLP024, one LTe20, one CL323a and one protected DNp03 output node. It contains 25 surviving selected edges.

On the 16-map ablation set, three LC4 neurons produced the largest measured individual effect: removing any one reduced success from 75% to 50%.

| FlyWire root ID | Type | Baseline | Ablated | Drop |
|---|---|---:|---:|---:|
| `720575940619397542` | LC4 | 75.0% | 50.0% | 25.0 pp |
| `720575940615575007` | LC4 | 75.0% | 50.0% | 25.0 pp |
| `720575940617176321` | LC4 | 75.0% | 50.0% | 25.0 pp |
| `720575940620729816` | LPLC4 | 75.0% | 62.5% | 12.5 pp |
| `720575940614572742` | LC4 | 75.0% | 62.5% | 12.5 pp |

These are task-specific ablation signals on a small synthetic benchmark, not evidence of biological necessity. DNp03 is protected by the implementation and is not counted as a valid ablation test.
## Interpretation

The experiment supports a restrained conclusion:

> The 377-neuron connectome-constrained controller contains substantial task redundancy under this UAV imitation objective. Learned gating identifies a sparse subset that is far more functional than same-size random subsets. Aggressive 24-neuron pruning retains useful behavior but shows a measurable generalization cost on an independent map set; a 32–40 neuron range appears more conservative in this experiment.

The result is therefore about **task compression inside a connectome-derived prior**, not about reproducing a fly brain or discovering a universal minimal biological circuit.

## Important limitations

- The student imitates `modified_fly`; it is not independently discovering an optimal avoidance policy from reward.
- Visual receptive-field assignment to LC4/LPLC2 nodes is synthetic and deterministic, not reconstructed from biological receptive-field measurements.
- The bilateral execution mirrors a shared hemisphere because the selected right DNp03 pathway is incomplete in this snapshot.
- Neurotransmitter sign handling is an engineering approximation; glutamatergic sign in particular is receptor/context dependent.
- Training and pruning are evaluated in the deterministic 2D arena, not the 6-DoF or PX4/Gazebo stack.
- The 24-node choice is exploratory and was chosen from a discrete pruning grid; the independent set showed an 8.3-point success decrease.
- The 32–40 neuron characterization is post-hoc on the confirmation set and needs a new untouched set before being treated as a confirmatory result.
- Random masks reuse the learned weights and are not retrained random-topology baselines.
- Sample sizes are modest and confidence intervals are wide.

## Final takeaway

The v0.2 extension strengthens the original feasibility study in a useful way: it shows that the connectome-derived topology can serve as a **structured prior for learning and pruning**, and that learned importance is meaningfully better than random deletion. It also demonstrates why an independent confirmation set matters: the most aggressive 24-neuron result was weaker out of sample than the selection set suggested.

For this repository, the experiment is considered complete at this point. Future work would require a separately scoped study with reward-based training, biological receptive-field constraints, larger untouched evaluation sets and eventual 6-DoF/PX4 deployment of the learned sparse controller.
