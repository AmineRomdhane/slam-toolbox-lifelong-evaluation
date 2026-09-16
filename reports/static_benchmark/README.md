# Static SLAM benchmark — World V0 / R1–R3

**SLAM Toolbox 2.8.5 · ROS 2 Jazzy · TurtleBot3 Burger · Gazebo Sim 8.11.0**

A static baseline for later dynamic-environment studies: compare trajectory accuracy,
map quality and process cost across three routes through World V0. Each route uses
one immutable recorded input and ten fresh SLAM processes. This measures
**fixed-input repeatability**, not ten independently simulated trajectories.

## Benchmark at a glance

**30/30 valid trials passed** the frozen execution checks (10/10 per route).
R1 validation pilots are excluded. R3 uses runs 001–009 plus 011: original 010 is
invalid because of concurrent external playback, with all evidence retained in the
[exclusion audit](data/r3_exclusion_audit.json). No experiments were rerun for this report.

| Route | Coverage intent | Nominal / canonical GT length (m) | Route / bag duration (s) | Input / reference scans | Valid cohort |
|---|---|---:|---:|---:|---|
| [R1](../../benchmark/routes/r1.yaml) | Horizontal rectangular loop | 9.50 / 9.578 | 159.866 / 170.884 | 855 / 854 | 004–013 |
| [R2](../../benchmark/routes/r2.yaml) | Complementary vertical/orthogonal coverage | 11.80 / 11.782 | 199.197 / 210.249 | 1052 / 1052 | 001–010 |
| [R3](../../benchmark/routes/r3.yaml) | Repeated central loop with more revisits | 9.50 / 9.539 | 193.312 / 204.313 | 1022 / 1022 | 001–009 + 011 |

## Setup and methodology

Acquire and validate one simulation-time MCAP per route, then freeze its hash.
Replay at 1× with rosbag2 as the sole 100 Hz clock authority into a fresh asynchronous
mapper. Independent Gazebo ground truth is evaluation-only; SLAM consumes laser
scans and robot TF. Save the online trajectory, final map/graph and SLAM-process
resource samples after each run.

Associate poses by exact simulation timestamp and fit one rigid **SE(2), without
scale correction**. Reuse that alignment for map scoring against a deterministic,
noise-free Gazebo reference at 0.05 m resolution, restricted to observed cells.
Resources cover active playback. See [methodology](../../docs/methodology.md),
[metric conventions](../../docs/metrics.md), [reference construction](../../docs/reference_maps.md)
and [frozen hashes/commits](../../docs/reproducibility.md#static-benchmark-provenance).

## Metrics

Let `e` be aligned XY error and `a` absolute wrapped yaw error. RMSE is
`√mean(error²)`; `Q` denotes a quantile. Exact association and endpoint rules are
specified in the linked metric conventions.

| Metric | Definition / formula | Units; interpretation |
|---|---|---|
| Success / coverage | Passed / valid trials; associated / canonical GT samples | %; higher, coverage qualifies accuracy |
| ATE / yaw APE | RMSE, median and ATE p95 of `e` / `a` | mm / mrad; lower global error |
| 1 m RPE | `F = (Gᵢ⁻¹Gⱼ)⁻¹(Eᵢ⁻¹Eⱼ)` at 1 m GT arc length; translation / yaw RMSE | mm / mrad; lower local drift |
| Occupied precision / recall | `TP/(TP+FP)` / `TP/(TP+FN)` | Fraction; higher correctness / recovery |
| F1 / IoU | `2TP/(2TP+FP+FN)` / `TP/(TP+FP+FN)` | Fraction; higher agreement |
| Boundary distance | Pooled bidirectional nearest-boundary distances; mean / median / p95 | mm; lower geometric discrepancy |
| CPU / RSS | `100 ΔCPU/Δwall`; resident memory; time-weighted mean / sampled peak | % of one CPU / MiB; process cost |
| Real-time factor | Simulation span / wall span | Ratio; playback pacing, not throughput |
| Repeatability | `dᵢ = 100(xᵢ/mean(x) − 1)`; `CV = 100 SD(x)/mean(x)` | %; smaller relative spread |

All aggregate entries below are **mean ± sample SD, n = 10**. Frozen JSON reports
retain every run, median/IQR, extrema and bootstrap 95% CIs (20,000 run-level
resamples). The Wilson 95% success interval for each 10/10 cohort is 72.25–100%.
Figures show all ten points, a thick IQR segment and a black median tick; horizontal
offsets only separate points. Each panel has its own labeled, zoomed y-axis.

## Trajectory results

| Metric | R1 | R2 | R3 valid |
|---|---:|---:|---:|
| Coverage (%) | 99.761 ± 0.000 | 99.895 ± 0.000 | 99.892 ± 0.000 |
| ATE RMSE (mm) | 11.700 ± 0.010 | 14.599 ± 0.011 | 10.908 ± 0.012 |
| ATE median (mm) | 12.387 ± 0.010 | 12.520 ± 0.044 | 10.559 ± 0.032 |
| ATE p95 (mm) | 17.503 ± 0.004 | 24.870 ± 0.006 | 17.457 ± 0.011 |
| Yaw APE RMSE (mrad) | 4.437 ± 0.006 | 4.476 ± 0.003 | 7.566 ± 0.009 |
| Yaw APE median (mrad) | 2.246 ± 0.012 | 1.800 ± 0.003 | 5.688 ± 0.009 |
| 1 m translation RPE RMSE (mm) | 13.744 ± 0.016 | 14.044 ± 0.027 | 16.487 ± 0.031 |
| 1 m rotation RPE RMSE (mrad) | 6.365 ± 0.005 | 3.428 ± 0.002 | 7.062 ± 0.005 |

![Run-level trajectory errors with median and IQR](assets/pose_distributions.png)
[Vector figure](assets/pose_distributions.svg)

## Map results

Each route has its own observed reference domain. Unknown reference cells are
excluded; unknown/out-of-map SLAM cells count as not occupied. Map alignment is
never refitted. Boundary distances pool both directions with point-count weighting.

| Metric | R1 | R2 | R3 valid |
|---|---:|---:|---:|
| Precision | 0.849835 ± 0.000000 | 0.837539 ± 0.000000 | 0.830225 ± 0.000834 |
| Recall | 0.607311 ± 0.000000 | 0.611751 ± 0.000000 | 0.581961 ± 0.000585 |
| F1 | 0.708391 ± 0.000000 | 0.707057 ± 0.000000 | 0.684270 ± 0.000688 |
| IoU | 0.548456 ± 0.000000 | 0.546859 ± 0.000000 | 0.520070 ± 0.000794 |
| Boundary mean (mm) | 24.237 ± 0.000 | 30.198 ± 0.000 | 23.797 ± 0.063 |
| Boundary median (mm) | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| Boundary p95 (mm) | 50.000 ± 0.000 | 50.000 ± 0.000 | 50.000 ± 0.000 |

![Run-level map quality with median and IQR](assets/map_distributions.png)
[Vector figure](assets/map_distributions.svg)

## System results

CPU and RSS refer only to the SLAM process during active playback. Peaks are
sampled at approximately 0.5 s intervals; 100% CPU represents one logical CPU.

| Metric | R1 | R2 | R3 valid |
|---|---:|---:|---:|
| Active CPU mean (%) | 4.863 ± 0.396 | 4.889 ± 0.521 | 5.206 ± 0.315 |
| Active CPU peak (%) | 14.366 ± 0.780 | 14.364 ± 1.220 | 14.755 ± 0.978 |
| Active RSS mean (MiB) | 46.420 ± 0.255 | 46.953 ± 0.371 | 46.681 ± 0.458 |
| Active RSS peak (MiB) | 46.821 ± 0.302 | 47.337 ± 0.390 | 47.009 ± 0.503 |
| Real-time factor | 0.999998 ± 0.000002 | 1.000001 ± 0.000001 | 1.000001 ± 0.000001 |

![Run-level active process resources with median and IQR](assets/system_distributions.png)
[Vector figure](assets/system_distributions.svg)

## Repeatability and diagnostics

Relative deviations below normalize each metric by its own route mean; labels
report sample CV. These are presentation summaries of the same ten frozen values,
not new experimental metrics or changed evaluation rules. CPU varies more than
pose scores; RSS varies less than CPU. R1/R2 map scores are identical across runs.

![Relative run-level deviations and route CVs](assets/repeatability.png)
[Vector figure](assets/repeatability.svg)

Constant diagnostics remain in the table. **Graph nodes are not processed scans**;
observed scans come from an independent topic monitor. Throttled MessageFilter
logs do not give exact scan-rejection totals.

| Metric | R1 | R2 | R3 valid |
|---|---:|---:|---:|
| Graph nodes | 12 ± 0 | 13 ± 0 | 10 ± 0 |
| Graph edges | 14 ± 0 | 14 ± 0 | 13 ± 0 |
| Observed scans | 855 ± 0 | 1052 ± 0 | 1022 ± 0 |
| MessageFilter drop log lines | 1 ± 0 | 0 ± 0 | 0 ± 0 |
| Queue-full log lines¹ | 0 ± 0 | 0 ± 0 | 0 ± 0 |

¹ Queue-full lines are a subset of MessageFilter lines, not additional rejections.

## Cross-route interpretation

- **Global accuracy and local drift differ.** R3 has the lowest translational ATE,
  but the highest yaw APE and both 1 m RPE errors. R2 has the highest ATE yet the
  lowest rotational RPE. Revisits alone neither guarantee accuracy nor prove loop closure.
- **Map scores describe different properties.** R1/R2 have similar F1; R2 recovers
  slightly more occupied truth but has larger mean boundary distance. R3 has lower
  F1 despite the smallest mean boundary distance. Observed domains differ by route.
- **Execution is stable for fixed inputs.** Pose-score spread is small relative to
  route differences; resource variability is more noticeable. This is descriptive,
  not a causal test of route geometry or a universal compute requirement.

## Limitations

One static world and one simulated realization per route limit generalization.
Errors are conditional on coverage and use online TF, not retrospectively optimized
trajectories. R1's first scan precedes odometry TF by about 20 ms; it also lacks a GT
interpolation bracket and is omitted only from reference generation. Startup data
remain unchanged. Exact processed/skipped scan counts and loop-closure events are
unavailable from current interfaces. The reference retains Gazebo rendering precision
and 5 cm grid quantization; grid phase affects map scores. Host-dependent resources
and sampled peaks are not hardware-independent measures. Runtime success alone
does not establish dataset validity, as the excluded R3 trial demonstrates.

## Data and figure reproduction

Pose/system: [R1](data/r1_pose_system.json), [R2](data/r2_pose_system.json),
[R3 valid](data/r3_pose_system.json). Maps: [R1](data/r1_map.json),
[R2](data/r2_map.json), [R3 valid](data/r3_map.json).
The [source manifest](data/source_manifest.json) preserves original paths and hashes;
[reproducibility documentation](../../docs/reproducibility.md) records frozen identity
and the exclusion audit. Raw experiment artifacts remain unchanged and gitignored.

From the repository root, regenerate only PNG/SVG figures with NumPy/Matplotlib:

```bash
python3 reports/static_benchmark/render_plots.py
```
