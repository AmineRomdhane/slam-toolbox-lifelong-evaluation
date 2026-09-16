# Metrics and statistical conventions

## Trajectory association and errors

The estimate is online map->base_footprint TF sampled at incoming GT stamps.
The evaluation reference is all /ground_truth/pose messages decoded directly from
the immutable MCAP, including both stationary intervals. This full reference is
used even if a failed run logged only a prefix. All distances use XY; full GT
XYZ/quaternion remains in the raw trajectory. No retrospective graph smoothing.

Parse decimal timestamp strings to integer nanoseconds using Decimal. Associate
only exact equal timestamps, one-to-one. Reject duplicate/nonincreasing estimate
times or nonfinite values. No ATE interpolation, nearest-neighbor tolerance,
time-offset fitting, extrapolation, clipping or outlier removal.

Primary trajectory_coverage = exactly associated samples / full canonical GT
samples. Also report covered-time fraction: sum of canonical adjacent time intervals
whose endpoints both have estimates, divided by full canonical duration. Accuracy
is conditional on coverage; never exclude a failed run from the run-level dataset.

Fit one rigid SE(2) from all associated estimated XY samples to GT by equally
weighted least-squares positional alignment. Rotation theta=atan2(sum(ex*gy-ey*gx),
sum(ex*gx+ey*gy)) on centered coordinates; translation aligns centroids. Fixed scale
1, no reflection. Yaw does not influence the fit. Fewer than two poses or degenerate
cross covariance makes alignment unavailable. Aligned yaw=wrap(estimated yaw+theta).
Absolute translation error is XY Euclidean distance; yaw error is the wrapped angle
in [-pi,pi]. Report translation RMSE/median/p95 and absolute-yaw RMSE/median (plus p95).
Percentiles use linear interpolation at (N-1)*p; all samples have equal weight.

Fixed-distance RPE: delta=1 m along cumulative canonical GT XY arc length. Start at
every associated GT sample with at least 1 m remaining (overlapping pairs, including
stationary starts). Find the first GT segment crossing exactly s_start+1; interpolate
its endpoint by arc fraction, linearly for XY and along shortest wrapped yaw for
both GT and estimate at the same timestamp. Require every intervening canonical
sample to have an estimate and every GT time step <=0.1 s; do not bridge gaps.
For relative transforms Dg=G_i^-1 G_j and De=E_i^-1 E_j, error=Dg^-1 De. Translation
RPE is its translation norm; rotational RPE is absolute wrapped yaw. No global
alignment is needed for RPE because it cancels in the relative transforms. Report
RMSE/median/p95 in metres/radians. Individual absolute/RPE samples are CSV files.

## Resource windows

Read only the SLAM PID's /proc cumulative utime+stime and RSS, approximately every
0.5 wall seconds, with extra samples at lifecycle boundaries. No host-wide CPU,
child-process attribution, RSS peak claims between samples, or normalization by
CPU core count. 100% CPU means one fully used logical CPU.

Primary active playback: wall receipt time of first generated /clock through wall
receipt time of last /clock. Startup is sampled process start through first clock;
finalization is last clock through last live sample, including drain/artifact save
and observed lifecycle shutdown. Store bounds relative to runner monotonic start.
Whole-process sampled metrics cover the entire raw sample span separately; actual
process lifetime is separately timed (small creation/exit tails are unsampled).
Raw phase labels describe sampling instants; recorded clock boundaries determine
the final analysis windows, without overwriting raw samples.

For each raw interval, CPU rate is cumulative CPU delta / elapsed wall time. Clip
interval overlap to each phase; integrate CPU rate and left-held RSS with time
weights. Peak CPU is the maximum overlapping sample-interval CPU rate; peak RSS
uses endpoints of overlapping intervals. Boundary peaks can include values from
an interval straddling the boundary, at approximately 0.5 s resolution. No samples
are deleted or rewritten. Preserve resources.csv and derived resource_metrics.json.

## Offline map scores

Reuse each trial's trajectory alignment; do not fit a map alignment. Invert SE(2)
to sample containing SLAM cells at reference cell centers. Restrict scoring to
reference-observed cells. Unknown/out-of-map SLAM cells are not occupied (false
negative for occupied truth, true negative for free truth).

Precision = TP/(TP+FP), recall = TP/(TP+FN), F1 = 2TP/(2TP+FP+FN),
IoU = TP/(TP+FP+FN). Boundaries are 4-neighbor occupied cells adjacent to a
nonoccupied cell within the observed domain. Pool both directed nearest-neighbor
boundary-cell distances; report mean, median and p95. This is point-count weighting,
not equal weighting of directional means. Distances are quantized at 0.05 m.

## Aggregates

One value per valid trial; arithmetic mean, sample SD (ddof=1), linear quantiles,
median/IQR, min/max. Percentile 95% bootstrap intervals use 20,000 run-level
resamples with seed 20260915 + metric index. Success uses a Wilson 95% interval.
Ten repetitions share one canonical bag per route; intervals describe fixed-input
repeatability, not scene generalization. Coverage accompanies every accuracy result.
The [report snapshots](../reports/static_benchmark/data/source_manifest.json)
preserve all individual values and calculation metadata.

## Report visualization and relative repeatability

Distribution panels show all ten valid run values with deterministic horizontal
spacing, the linear-quantile interquartile range (Q1–Q3), and median. No density
estimate or histogram is fitted. Units are separate by panel; axes are zoomed.
Equal values remain ten visible horizontally separated points. Constant diagnostics
are tabulated instead of plotted.

For strictly positive ratio-scale metrics (ATE RMSE, translational RPE RMSE,
active CPU mean and active RSS mean), the repeatability figure displays each run's
signed relative deviation d_i = 100 * (x_i / arithmetic_mean(x) - 1).
Labels show CV = 100 * sample_SD(x, ddof=1) / arithmetic_mean(x).
Each route is normalized independently. The median/IQR uses these transformed
run-level values. CV is a cohort summary, not a per-run measurement. Near-zero,
constant diagnostic and bounded map-score CVs are not used. These presentation-only
calculations do not replace or modify frozen aggregates, association, alignment,
resource windows or experiment outputs.
