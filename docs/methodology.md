# Benchmark methodology

Static World V0 benchmark with SLAM Toolbox 2.8.5 on ROS 2 Jazzy.
The [static report](../reports/static_benchmark/README.md) is the authoritative human-readable result.

## Execution

Input MCAP SHA256: f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715.
Verify before running and after completion. No bag modification, simulator, route
execution, or automatic retries. Each output directory is exclusive. The accepted validation pilot is run_003; pilots are excluded from the R1 baseline.

Relative to installed SLAM Toolbox 2.8.5 async YAML, only use_sim_time=true and
max_laser_range=3.5 differ. Relative to pilot 1, only max_laser_range changes.
Manifest hashes freeze exact configuration bytes. Run metadata and a tooling
snapshot record code/configuration hashes, Git commit and working-tree status.
SLAM is lifecycle-configured/activated before rosbag2 plays the five input topics
at 1x with a sole generated 100 Hz clock and 3 s launch delay. After playback,
allow 6 wall seconds for final processing without creating another clock; then
save the grid via GetMap, serialize graph/data and stop SLAM/monitors.

## Scan eligibility and observability

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

## Success and retention

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

The original R3 run_010 was subsequently adjudicated invalid due to confirmed
concurrent external playback. One explicitly authorized replacement, run_011,
is included; original results and success fields remain untouched. See the
[cohort audit](../reports/static_benchmark/data/r3_valid_cohort.json).

## Clock guard

Query /clock before playback and repeatedly during playback. Count publisher GIDs
separately from node identity. Zero publishers means no source discovered yet. One
rosbag2_player endpoint is expected; one unresolved identity warns once per GID and
continues. Multiple simultaneous endpoints or one clearly named unexpected source
fails. Name resolution on the same GID is not a second source. Log all snapshots.
Verify clock advancement after player launch: allow 12 wall seconds for the existing
3 s playback delay/startup, then fail if advancement stops for over 2 wall seconds
while the player is active. Existing backward-time checks remain. The guard only
reads discovery/clock data and never terminates external processes.

The guard does not detect external TF-only publishers. Replacement preflight and
process observations are preserved in its audit; this restructuring does not
change the guard. Metric definitions are in [metrics.md](metrics.md); execution
provenance and layout compatibility are in [reproducibility.md](reproducibility.md).
