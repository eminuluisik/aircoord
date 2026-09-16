# AIRCOORD
## Delayed-State Compensation for Multi-Vehicle Coordination with Centralized MPC
Technical demo report | 16 September 2026

### Abstract

AIRCOORD studies the effect of delayed state information on centralized coordination in a planar four-vehicle crossing scenario. The controller combines nominal goal tracking with nonlinear model predictive control (MPC), soft separation constraints and command-history-based state reconstruction. Three information conditions are compared: current reports, stale reports and compensated reports. The supplied experiment grid uses ten shared scenario seeds, report delays of 0.5, 1 and 2 seconds, and independent plant acceleration disturbances with standard deviation 0.3 m/s². At 2 seconds, stale-state MPC records separation violations in 7/10 episodes and 60% vehicle arrival, whereas compensation records 0/10 episodes with violations and 100% arrival. The evidence supports a focused prototype result under the tested assumptions. It does not establish formal safety, real-time feasibility or deployment readiness.

## 1. Research question and contribution

State reports can arrive after vehicles have moved. Applying a controller directly to an old report can produce commands for a situation that no longer exists. AIRCOORD asks whether replaying the commands sent since the report timestamp can reduce that mismatch, even when the plant experiences disturbances unavailable to the estimator.

The contribution is an inspectable simulation demo: a delayed-information interface, a model-based compensation mechanism, a shared controller and a small controlled comparison. The work demonstrates experimental reasoning about information age, control and evaluation. Novelty relative to the wider literature has not been established. The implementation contains no learned policy or human interaction model.

## 2. Scenario and dynamics

Four vehicles begin near evenly spaced points on a circle of radius 150 m and travel toward opposite nominal points. Each initial coordinate receives uniform jitter in [-5, 5] m. Seeds 7-16 vary those initial positions and the disturbance realizations; they do not generate new families of traffic geometry.

The state of vehicle i is (x, y, speed, heading); its control is (longitudinal acceleration, heading rate). At each 0.1 s step, the simulator clips the command, updates speed and heading, then advances position using the updated velocity. Plant acceleration adds an independent Gaussian disturbance after command saturation. Thus total acceleration can exceed the actuator-command limit; speed still remains bounded.

| Parameter | Value |
|---|---|
| Vehicles / simulation limit | 4 / 60 s |
| Simulation step | 0.1 s |
| Initial radius / position jitter | 150 m / ±5 m per coordinate |
| Cruise / maximum speed | 10 / 15 m/s |
| Command acceleration bound | ±2 m/s² |
| Heading-rate bound | ±30 degrees/s |
| Required separation / goal radius | 20 m / 5 m |
| Plant acceleration disturbance | Gaussian, std 0.3 m/s² per vehicle and step |
| MPC horizon / constant-control blocks | 5 s / 5 |
| Additional separation margin | 3 m |
| Optimization limit / initialization | 80 SLSQP iterations / warm |

The disturbance is generated using a separate seeded stream indexed by time step and vehicle. Corresponding runs receive the same disturbance sequence, independently of solver calls and vehicle arrival. This is a controlled comparison mechanism, not a calibrated atmospheric turbulence model. Vehicles leave the active traffic set at the end of the first step whose endpoint is within the goal disk.

## 3. Controller and information conditions

### 3.1 Nominal tracking and centralized MPC

The nominal policy steers toward the goal and regulates speed toward 10 m/s. It does not avoid other vehicles. The MPC filter jointly adjusts the controls of all active vehicles. Each five-second plan contains five constant-control blocks per vehicle; only the first simulation-step command is executed before replanning.

Commands are normalized by actuator limits. The objective combines mean squared deviation from the nominal command, a control-smoothing term weighted by 0.1, and a global slack penalty of 1000(s + s²). The nominal command is held constant over the prediction horizon. Separation constraints use the target D = 20 + 3 = 23 m and have the form:

    predicted closest distance / D - 1 + s >= 0, with 0 <= s <= 1

Slack permits violations of the nominal target and, when sufficiently large, of the required 20 m separation. The reported slack in metres is D*s. A large penalty discourages slack but is not a proof of safety.

If the nominal plan already meets the target, optimization is skipped. Otherwise, SciPy SLSQP solves the joint nonlinear problem. An accepted solution must report success, remain finite and satisfy the soft constraints within the implementation tolerance. If rejected, the controller commands maximum braking while retaining the nominal turn rate. That fallback has no separation guarantee.

Warm initialization shifts the preceding plan by one simulation step and projects it back onto the block representation. The shifted plan is an initial guess, not a verified backup plan. Memory is cleared when an optimization is rejected or the nominal plan is used.

### 3.2 Current, stale and compensated reports

| Mode | Information supplied to MPC |
|---|---|
| current | True current simulator state and active mask; report age is zero. |
| stale | Historical state and active mask at the delayed timestamp. |
| compensated | Historical report advanced to the present by replaying sent commands with the nominal model. |

All dynamic fields in a central report share a timestamp. Fixed goals are mission information. At startup, unavailable history is clamped to reset time, so actual report age initially grows toward the configured delay.

Compensation uses the sent-command history and nominal dynamics. It does not read the realized acceleration disturbance or the true current state. Under exact disturbance-free dynamics it can reconstruct the present state; unobserved disturbances create reconstruction error. It also predicts arrival and deactivation, which can disagree with the true active mask under disturbance.

Compensation assumes known dynamics, reliable command history, immediate command execution and known goals. Only report delay is modeled. The current-state reference ignores the configured delay; its three delay rows are repeated evaluations of the same information condition.

## 4. Evaluation and evidence provenance

The principal analysis uses the nine ten-seed summaries from ver6: three report modes at three delay settings. This yields 90 run records, with ten shared seeds across conditions. The three current-state sets should not be treated as independent scenario samples.

The supplied archive also contains ver3-ver5, preliminary one-seed runs and some directories with additional delay subfolders. Those are not pooled into the main comparison. Conditions were identified from saved configuration and episode fields. Each selected summary episode was checked against its corresponding archived metrics.json: all 90 matched.

An episode has a separation violation if any pair of vehicles active at the beginning of a step approaches within 20 m during that step. Closest approach is evaluated along the simulator's linear step segments, not only at sampled endpoints. This is exact for that segment model, not for continuous curved aircraft motion.

Arrival rate is the fraction of the four vehicles that enter their goal disks within 60 simulated seconds. Worst separation is the smallest recorded pair distance over all episodes in a condition. Failed solver calls are rejected optimization attempts; a solver failure and a separation violation are different events. Arrival-time metrics exclude non-arriving vehicles and should not be used alone to compare incomplete runs.

## 5. Results

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

At 0.5 s delay, all three modes retain full arrival and no observed separation violations. At 1 s, stale-state control retains separation in these samples but arrival falls to 97.5% (39 of 40 vehicles). This matters: absence of separation violations does not by itself imply successful coordination.

At 2 s, stale-state control degrades substantially: 7/10 episodes violate separation, only 24 of 40 vehicles arrive, and the worst observed pair distance is 3.980 m. Compensation restores full arrival in this sample, with a worst separation of 22.685 m and no recorded episode violation. Relative to stale reports, this is a 70 percentage-point reduction in the observed episode violation rate and a 40 percentage-point increase in vehicle arrival.

These comparisons are descriptive. Ten seeds are insufficient to characterize rare failures or establish general reliability. No formal hypothesis test or out-of-distribution claim is made. For perspective, even 0 failures in 10 independent Bernoulli trials has a one-sided 95% upper failure-probability bound of about 25.9%; the present shared-scenario experiment is more restricted than a broad deployment sample.

### Computational behavior

Archived maximum single-decision times exceed the 0.1 s simulation step. For the 2 s stale condition, the largest recorded decision time is about 28.4 s. The simulator waits for the controller; solver latency does not age the state or advance the plant while optimization runs.

Repeated current-state groups have identical trajectory metrics but substantially different recorded wall times. Hardware and load controls are not documented. Consequently, these records do not support a controlled speedup claim or real-time feasibility claim. Future timing experiments should record hardware, library versions, process load, execution order and deadline-miss rates.

## 6. Independent local verification

The distributed implementation preserves the supplied ver6 algorithm. Four focused tests passed: between-endpoint crossing detection, exact compensation without disturbance, nonzero estimation error with unobserved disturbance, and zero report age in current-state mode.

Three complete episodes were rerun locally using seed 7, delay 2 s, noise std 0.3 m/s² and warm initialization. Current-state MPC achieved minimum separation 22.998935 m and 100% arrival; stale-state MPC achieved 6.822961 m and 75% arrival with a violation; compensated MPC achieved 22.794663 m and 100% arrival without a violation. These values and solver-failure counts matched their archived records.

Local environment: Python 3.10.7, NumPy 1.26.4, SciPy 1.15.3, Matplotlib 3.10.0 and Pillow 12.0.0. The original experiment environment is not established by the supplied metadata. Timing differed locally. The full 90-run grid was audited but not rerun.

## 7. Limitations and next experiments

The current study is a small planar crossing experiment with centralized access to vehicle reports. It does not address realistic aircraft dynamics, three-dimensional separation, sensing error, packet loss, variable delays, command delay or decentralized communication. The optimizer is nonconvex, uses soft constraints and has an unverified fallback. Solver success does not establish real-world safety.

The next useful study would extend the existing controlled design:

1. Sweep traffic density and crossing geometry, using held-out seeds and a wider range of encounter angles.
2. Sweep delay and disturbance intensity, including biased disturbances and model mismatch; measure state-reconstruction and active-mask errors.
3. Compare command replay with a simpler constant-velocity reconstruction and an uncertainty-aware estimator.
4. Integrate solver and command-execution latency into the plant timeline, then report deadline misses and fallback outcomes.
5. Evaluate uncertainty-dependent margins and a separately validated fallback, reporting safety and completion together.

A learned residual model, decentralized policy or human oversight interface could be a later extension. None is part of the current implementation.

## 8. Relevance to the prospective research group

Bizhao Pang's group describes interests in decision-making under uncertainty, human-AI collaboration and multi-agent systems, and trustworthy AI for aviation. AIRCOORD's most direct connection is to uncertainty-aware coordination and transparent evaluation of safety-related behavior. Human-AI collaboration is a future direction rather than a demonstrated component.

A suitable project description for an application is:

> I developed AIRCOORD, a reproducible simulation prototype for studying delayed-state information in centralized multi-vehicle coordination. I compared stale reports with command-history-based compensation under controlled acceleration disturbances, evaluated separation and completion, and documented solver failures and timing limitations. I would like to extend this work toward uncertainty-aware coordination and more realistic aviation scenarios.

This describes the implemented work without claiming affiliation, published novelty or certification.

## References and accompanying material

[1] Supplied aircoord.rar: ver6/aircoord.py and the nine ver6 ten-seed summary sets. The accompanying evidence directory preserves the selected records; docs/VALIDATION.md describes the audit.

[2] Bizhao Pang, research group description and application information. https://bizhao001.github.io/ (accessed 16 September 2026). Used for research-fit context only.

See README.md for installation and example runs; scripts/run_experiments.py specifies the complete grid; scripts/summarize_evidence.py regenerates the main table. Author name and publication license should be finalized before public release.

