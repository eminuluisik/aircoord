# AIRCOORD
### Delayed-state compensation for multi-vehicle coordination with centralized MPC

AIRCOORD is a small 2D simulation study of how delayed state reports affect cooperative trajectory control. A centralized model predictive controller (MPC) compares fresh reports, stale reports, and estimates reconstructed by replaying previously sent commands.

The main experiment asks: **Can command replay recover coordination performance when state reports are delayed and the plant experiences unobserved acceleration disturbances?**

## Main finding

In the supplied four-vehicle crossing experiments with 2 s report delay and acceleration disturbance standard deviation 0.3 m/s², stale-state MPC recorded separation violations in **7 of 10 episodes** and a **60% vehicle arrival rate**. Command-replay compensation recorded **0 of 10 episodes with violations** and **100% arrival** on the same seeds. These are archived simulation results, not a safety guarantee.

## What is implemented

- A planar kinematic simulator with speed, acceleration and turn-rate limits.
- A nominal goal-tracking baseline.
- Joint nonlinear MPC, solved with SciPy SLSQP, with soft separation constraints.
- Cold initialization and shifted warm initialization.
- Timestamp-consistent current/stale reports and command-replay compensation.
- Plant-only acceleration disturbances, with a separate seeded random stream.
- Closest-approach checks throughout each linear simulation step.
- Per-step solver diagnostics, episode metrics, trajectory plots and optional GIF replay.

This repository contains no reinforcement-learning policy, decentralized controller, human-in-the-loop study or flight validation.

## Representative trajectories

Seed 7, 2 s report delay, acceleration noise std 0.3 m/s². These are the supplied archived plots. The left panels show complete trajectories with initial vehicle markers; right panels show closest separation over time.

**Stale reports:** separation violation; 3 of 4 vehicles arrive.

![Stale-state MPC trajectory and separation](assets/stale.png)

**Compensated reports:** no observed separation violation; all 4 vehicles arrive.

![Compensated MPC trajectory and separation](assets/compensated.png)

## Quick start

Python 3.10 is the locally checked interpreter. Run commands from this directory.

```bash
python -m venv .venv
```

Activate the environment using `.venv\Scripts\Activate.ps1` on Windows PowerShell or `source .venv/bin/activate` on macOS/Linux, then:

```bash
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python aircoord.py --controller baseline --seed 7 --out results/baseline --gif
python aircoord.py --controller mpc --initialization warm --state-source compensated --delay 2 --accel-noise-std 0.3 --seed 7 --out results/compensated --gif
```

MPC may run slower than simulated time. For a quick inspection of one optimization call:

```bash
python aircoord.py --inspect-mpc --inspect-at 10 --initialization warm --state-source compensated --delay 2 --accel-noise-std 0.3 --out results/inspection
```

## Reproduce the experiment grid

```bash
python scripts/run_experiments.py
```

This launches nine conditions, each using seeds 7–16 (90 runs). It can take substantial time. Each condition has its own output directory; use a new output root to preserve earlier runs:

```bash
python scripts/run_experiments.py --out results/new_grid
```

To inspect the nine commands without running simulations:

```bash
python scripts/run_experiments.py --dry-run
```

Regenerate the evidence table, without running the controller:

```bash
python scripts/summarize_evidence.py
```

## Archived results

| Delay setting (s) | State source | Episodes with violation | Arrival rate | Worst separation (m) | Failed solver calls |
|---:|---|---:|---:|---:|---:|
| 0.5 | current | 0/10 | 100.0% | 22.896 | 1 |
| 0.5 | stale | 0/10 | 100.0% | 21.399 | 10 |
| 0.5 | compensated | 0/10 | 100.0% | 22.938 | 1 |
| 1.0 | current | 0/10 | 100.0% | 22.896 | 1 |
| 1.0 | stale | 0/10 | 97.5% | 23.692 | 26 |
| 1.0 | compensated | 0/10 | 100.0% | 22.865 | 0 |
| 2.0 | current | 0/10 | 100.0% | 22.896 | 1 |
| 2.0 | stale | 7/10 | 60.0% | 3.980 | 34 |
| 2.0 | compensated | 0/10 | 100.0% | 22.685 | 1 |

Required separation is 20 m; the planner targets 23 m before slack. Each condition contains ten episodes with four vehicles. Arrival rate is the fraction of vehicles arriving within the 60 s simulation limit. A solver failure is a rejected optimization call, not necessarily an episode separation violation.

**The current-state reference ignores the delay setting and has zero report age.** Its repeated delay rows are the same information condition, not three independent safety evaluations. The nine evidence files contain 90 run records but only ten distinct scenario seeds.

## Repository guide

- `aircoord.py`: supplied `ver6/aircoord.py` implementation, with no algorithm changes.
- `requirements.txt`: dependency versions available in the local verification environment.
- `tests/test_core.py`: geometry and report-compensation checks.
- `scripts/run_experiments.py`: explicit experiment grid.
- `scripts/summarize_evidence.py`: regenerates the results table from saved records.
- `evidence/*.json`: supplied summary records, organized by experiment condition.
- `docs/VALIDATION.md`: provenance and verification status.

A normal run saves `metrics.json`, `diagnostics.csv`, `trajectory.npz`, `overview.png` and, with `--gif`, `replay.gif`. The output hierarchy includes controller, report mode, delay, initialization and seed.

## Scope and limitations

Soft constraints and braking fallback do not guarantee separation. The simulator waits for optimization to finish; report delay does not model solver latency. Disturbance magnitude and four-way crossing geometry are limited. Current reports are an ideal reference, and compensation assumes known dynamics and reliable immediate execution of sent commands. See [validation and provenance](docs/VALIDATION.md) for the checks performed and their scope.

## Research context

This independent demo relates to uncertainty-aware aircraft coordination and trustworthy decision support. These themes overlap with the [Human–AI Collaborative Intelligence for Aviation Systems group](https://bizhao001.github.io/). This project is not affiliated with or endorsed by the group and does not reproduce a specific published method.

## License

A license has not yet been selected for this repository.
