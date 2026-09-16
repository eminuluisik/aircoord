# Provenance and validation

Prepared 16 September 2026 from the supplied `aircoord.rar` archive.

## Included material

- The runnable implementation is the archive's `ver6/aircoord.py`; algorithm behavior was not edited. The distributed text has normalized line endings.
- The nine evidence JSON files correspond to `ver6/results_noise03_delay{05,1,2}_10seeds_{current,stale,compensated}/summary.json`. JSON formatting was normalized, with values preserved.
- Older versions (ver3-ver5), preliminary one-seed runs and miscellaneous nested runs are not pooled into the main analysis. Some archive directories contain runs for more than one delay; condition selection follows each summary's configuration and episode records, not folder names alone.
- All 90 selected episode records were compared against their matching archived `metrics.json` files. All matched exactly.
- The original archive does not establish the original Python/library versions, hardware specification or source commit hash. The requirements file records locally verified versions, not a reconstructed original lockfile.

## Local checks

Environment: Windows, Python 3.10.7; NumPy 1.26.4, SciPy 1.15.3, Matplotlib 3.10.0, Pillow 12.0.0.

Four tests passed: detection of between-endpoint crossings; exact command replay in a disturbance-free plant; nonzero reconstruction error under unobserved plant disturbance; zero measurement age in current-state mode.

Three full MPC episodes were rerun with seed 7, 2 s delay, 0.3 m/s² acceleration noise and warm initialization:

| Mode | Minimum separation (m) | Arrival rate | Violation | Failed solver calls |
|---|---:|---:|---|---:|
| current | 22.998935 | 100% | No | 0 |
| stale | 6.822961 | 75% | Yes | 5 |
| compensated | 22.794663 | 100% | No | 0 |

These separation, arrival and failure values matched the corresponding archived seed-7 results. Wall-clock timings differed, as expected. All 90 experiments were not rerun; the main table is an audited summary of supplied data.

## Remaining research validation

The unit checks are focused and do not certify correctness of all simulator or optimization paths. Further validation should cover packet loss, variable delays, command execution delays, active-mask prediction errors near goals, larger traffic patterns, additional noise levels and controlled timing benchmarks. Use a fresh output directory when generating results to avoid confusing files from earlier conditions.
