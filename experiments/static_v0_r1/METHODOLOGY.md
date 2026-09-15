# V0/R1 methodology v2

## Input, configuration and execution

Input MCAP SHA256: f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715.
Verify before running and after completion. No bag modification, simulator, route
execution, or automatic retries. Each output directory is exclusive. The revised
pilot is run_002; run_001 and its raw samples remain unchanged.

Relative to installed SLAM Toolbox 2.8.5 async YAML, only use_sim_time=true and
max_laser_range=3.5 differ. Relative to pilot 1, only max_laser_range changes.
Manifest hashes freeze exact configuration bytes. Run metadata and a tooling
snapshot record code/configuration hashes, Git commit and working-tree status.
SLAM is lifecycle-configured/activated before rosbag2 plays the five input topics
at 1x with a sole generated 100 Hz clock and 3 s launch delay. After playback,
allow 6 wall seconds for final processing without creating another clock; then
save the grid via GetMap, serialize graph/data and stop SLAM/monitors.

## Scan terminology and observability

- Bag input: 855 scans, established from canonical bag information.
- Input-topic monitor: number received independently by the runner on /scan.
  This is not a counter inside SLAM's subscription or laserCallback.
- SLAM callback entries / scans passed to Karto: exact counts unavailable through
  the default public interfaces and logs used here.
- Explicit TF rejection: preserve each logged stamp/reason. MessageFilter logs
  are throttled at 2500 ms, so observed log count is a lower bound, not an exact
  rejection count. Pilot 1 explicitly rejected scan 11.000 s (before TF cache).
- Eligibility skips: exact time/motion/initial-stabilization counts unavailable.
- Insertions: /pose publications occur after successful Mapper::Process and
  Dataset::Add; final graph markers independently give graph node count. Pilot 1
  had 12 insertions and 12 nodes; neither means only 12 scans were processed.
  Graph edges come from LINE_LIST point pairs. Automatic loop-closure events and
  edge types/information matrices are unavailable from these marker/default logs.

Inspected 2.8.5 code: src/slam_toolbox_async.cpp::laserCallback;
src/slam_toolbox_common.cpp::shouldProcessScan and addScan;
lib/karto_sdk/src/Mapper.cpp::Process and HasMovedEnough;
installed tf2_ros/message_filter.hpp::signalFailure.

Eligibility parameters and behavior:

| Parameter | Value | Role |
|---|---:|---|
| throttle_scans | 1 | No modulo decimation |
| minimum_time_interval | 0.5 s | Wrapper elapsed-time gate |
| minimum_travel_distance | 0.5 m | Wrapper and Karto movement gates |
| minimum_travel_heading | 0.5 rad | Karto heading gate; wrapper uses it only in precise mode |
| check_min_dist_and_heading_precisely | false | Wrapper rejects squared displacement <0.8×0.5² (distance <0.447214 m) |
| scan_queue_size | 1 (effective default) | TF MessageFilter capacity |
| transform_timeout | 0.2 s | TF filter/transform handling |
| tf_buffer_duration | 30 s | Transform cache |
| min_laser_range / max_laser_range | 0 / 3.5 m | Laser range bounds |
| use_scan_matching | true | Karto matching/graph processing |
| paused_new_measurements | false | Measurement gate remains open |

First eligible measurement bypasses wrapper time/motion gates; callback counters
2–4 fail initial stabilization if reached. The wrapper updates its last eligible
pose/time before calling Karto, even if Karto subsequently rejects insertion.
Karto checks movement against the previous stored scan (0.5 m OR 0.5 rad).
It also accepts a 3600 s elapsed interval by its internal MinimumTimeInterval
default; this is distinct from the wrapper ROS minimum_time_interval=0.5 s and
is not reached during this bag.
Consequently, input count minus graph size must not be labeled a processed/skip
counter. Other scan matcher and loop parameters remain installed defaults.

## Trajectory association, alignment and errors

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

## Process resources and phase boundaries

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

## Success and failure retention

Run-level fields: success, trajectory_coverage, failure_reason, failure_sim_time
(simulation seconds), failure_route_fraction. On success all three failure fields
are null. On failure record the detected time and reason, retain partial outputs,
and evaluate coverage against the full canonical reference. If time is unavailable
before playback, failure_sim_time=null and route fraction=0. Fraction is canonical
GT XY arc progress at the failure time between dataset metadata's actual route
start/end; clamp to [0,1]. This measures reference progress, not planned time ratio.

Success requires normal execution/shutdown, scans and insertions observed, map and
graph saved, valid trajectory/RPE evaluation, resource phase separation, unchanged
input/configuration, exclusive replay clock, GT evaluation-only and no obsolete
20 m warning. No accuracy threshold is invented; startup missing TF is documented
in coverage, not filled. A failing run remains a failed result even if partial
trajectory accuracy can be calculated. Preflight refusal before run allocation is
not a trial. No automatic replacement runs are allowed.
