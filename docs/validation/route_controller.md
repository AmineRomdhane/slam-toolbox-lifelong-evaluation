# Final-pose correction: three fresh-world runs

All three runs of the final implementation completed within the configured
0.015 m position and 0.015 rad yaw tolerances, in the same terminal GT sample.
The controller, independent GT extractor and unchanged official TurtleBot3 World
were freshly launched for each trial and stopped afterward. No SLAM or rosbag.

## Change

Final yaw completion now rechecks position and reuses ROTATING/TRANSLATING at the
existing last waypoint if necessary, followed by final yaw restoration. Corrective
translations target 10% of the configured position tolerance (1.5 mm) to leave
margin for measured rotation-induced displacement. Ordinary waypoint tolerances,
R1 geometry/YAML, velocity and acceleration limits are unchanged. Corrective
rotation must finish before translation; passing near the waypoint during rotation
does not shortcut that phase. All corrections share the original 90 simulation
second final-phase deadline, starting at first final yaw rotation. Existing safety
aborts remain active. No new parameters, subscriptions, TF or simulator changes.

The harness accepts an output directory and run count; analysis discovers trial
directories. The README is shortened. Thirteen core tests pass, covering convergence,
correction margin, rotation-phase transition, shared timeout, safety aborts and
bounded acceleration. Only slam_eval_route_controller was built.

## Measurements

| Run | Duration (sim s) | Path (m) | Max tracking (mm) | Mean tracking (mm) | Final position (mm) | Final yaw (rad) | Abort |
|---|---:|---:|---:|---:|---:|---:|---|
| 1 | 160.419 | 9.581171 | 26.505 | 11.482 | 13.948 | 0.014089 | No |
| 2 | 160.432 | 9.581305 | 26.650 | 11.503 | 13.948 | 0.014188 | No |
| 3 | 160.460 | 9.581689 | 26.855 | 11.524 | 13.957 | 0.014029 | No |

Tracking error remains distance to the active finite nominal segment; its mean
includes turns and final corrections. Final XY error is relative to (-2,-0.5),
yaw relative to 0. Full actual start poses and simulation stamps are saved per run.
These are terminal-sample tolerances, not a measured post-shutdown dwell guarantee.

Pairwise XY comparison uses 1001 interpolated samples in the unchanged Gazebo world
frame with no spatial registration, including correction motions:

| Pair | Elapsed-time RMS (mm) | Elapsed-time max (mm) | Arc-length RMS (mm) | Arc-length max (mm) |
|---|---:|---:|---:|---:|
| 1–2 | 1.302 | 3.396 | 0.079 | 0.168 |
| 1–3 | 3.297 | 6.083 | 0.292 | 0.543 |
| 2–3 | 2.123 | 3.293 | 0.253 | 0.423 |

Elapsed-time alignment uses the common 160.419 s; normalized arc length compares
all of each trajectory. Duration spread is 0.041 s; path-length spread 0.518 mm.
Recorded command acceleration maxima were 0.10 m/s² and 0.30 rad/s² within numerical
roundoff. Conservative geometric screening found no obstacle intersection; minimum
between-sample clearance bound was 0.178236 m for the 0.20 m robot envelope.
Physics contacts were not instrumented.

Evidence: `~/slam_testing/results/r1_final_pose_04/summary.json`,
`source_sha256.json`, `run_1` through `run_3` trajectory/configuration/result files,
and process logs. Results are git-ignored and should be archived separately.

## Development attempts (excluded from final-version comparison)

Three earlier versions each hit the preserved final-phase timeout; their full
logs and trajectories are retained in `r1_final_pose_01`, `_02`, `_03`.
The first reused the 15 mm translation tolerance and failed to leave rotation
margin. The second used 3 mm but allowed a premature rotation-phase shortcut.
The third removed that shortcut but settled near 15.03 mm after yaw restoration,
just outside acceptance. The final version uses a 1.5 mm corrective target.
These failed development trials are not successful validation runs. The source
was unchanged across all three `_04` trials.

---

# Historical baseline (before final-pose correction)

# R1 five-run validation

All five measured trials completed. Each trial used a fresh official TurtleBot3
World process, fresh ground-truth extractor and fresh controller. No SLAM node,
canonical bag, scan-based control, odometry-based control or route replanning.

| Run | Duration (sim s) | XY path (m) | Max tracking (cm) | Mean tracking (cm) | Final XY error (cm) | Final yaw error (rad) | Abort | Geometric collision |
|---|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 144.178 | 9.539487 | 2.6554 | 1.2473 | 2.5419 | 0.014000 | No | No |
| 2 | 144.161 | 9.539557 | 2.6904 | 1.2542 | 2.5396 | 0.014227 | No | No |
| 3 | 144.151 | 9.539431 | 2.6673 | 1.2512 | 2.5361 | 0.014038 | No | No |
| 4 | 144.139 | 9.539402 | 2.6790 | 1.2525 | 2.5370 | 0.013975 | No | No |
| 5 | 144.159 | 9.539080 | 2.6762 | 1.2523 | 2.5364 | 0.014137 | No | No |

Tracking error is distance to the active finite nominal segment. Mean is the
sample mean over the full run, including turns. Final position is relative to
nominal (-2,-0.5); final yaw is relative to zero. The 15 mm position tolerance is
checked before final rotation; Gazebo translation during the turn increases
the final XY error to about 25 mm. No corrective segment was added.

Pairwise comparisons use 1001 interpolated XY samples in unchanged Gazebo world
coordinates (no spatial alignment), across all ten pairs:
- Elapsed-time alignment over common 144.139 s: pairwise RMS 0.521–4.550 mm;
  largest pointwise difference 6.314 mm.
- Normalized travelled-distance alignment: pairwise RMS 0.057–0.301 mm;
  largest pointwise difference 0.567 mm.
- Duration spread: 0.039 s. Path-length spread: 0.477 mm.

Every command trajectory respected the configured 0.10 m/s² linear and
0.30 rad/s² angular acceleration limits within floating-point error.
Controller ROS subscriptions were verified as /ground_truth/pose and /clock only.
Status was observed on /route/status; velocity type is TwistStamped.

Collision status is conservative geometric screening, not a physics contact
measurement: the world has no contact-reporting sensor. Minimum 0.20 m circular
envelope clearance over all sampled trajectories was 0.1800 m; subtracting the
largest step provides a straight-interpolation lower bound of 0.1782 m. No
post, hexagon or wall intersection was detected.

Results are in ~/slam_testing/results/r1_validation_03:
summary.json, per-run trajectory.csv/configuration.json/result.json, and process
logs. Exact actual start positions/quaternions/timestamps are retained per run.
Paths are in the repository's ignored results/ directory; keep them separately
when archiving the experiment. The analysis and trial harness are package scripts.

Setup attempts in r1_validation_01 and r1_validation_02 had no route motion and
are excluded. In-process Gazebo full reset removed Burger; subsequent respawn
caused an Ogre duplicate lidar-camera exception. Fresh-process world resets
avoided that problem without modifying world/model/simulator packages.

Core and YAML used throughout the five runs:
- control_core.py SHA256: 000d3a70377b2792745a1db819d3dc0ec7cb72acdb881bdbe02f136112ec8228
- r1.yaml SHA256: 9e9f8948fcc99fe6b5ec18c90bc0acf3b57959c412b20c9eb3c02e6af7246c78

After the runs, terminal status publication was also routed through finish()
so watchdog/interrupt aborts report ABORTED even without another GT callback.
This changes terminal logging only, not the validated command law. Final package
build and seven control tests passed. All trial processes were stopped.
