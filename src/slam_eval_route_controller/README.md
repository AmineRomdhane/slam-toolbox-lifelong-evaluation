# slam_eval_route_controller

Runs predefined YAML routes using `/ground_truth/pose` feedback and
`geometry_msgs/msg/TwistStamped` commands on `/cmd_vel`. No scan, odometry,
TF lookup, or SLAM feedback is used. R1 geometry is in `routes/r1.yaml`;
`route_file` selects another definition.

## Run

Start TurtleBot3 World and the ground-truth extractor first. In a clean shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/slam_testing/install/local_setup.bash
ros2 run slam_eval_route_controller route_controller --ros-args \
  -p use_sim_time:=true -p output_dir:=/tmp/r1_trial_unique
```

The output directory must not exist. It receives the exact actual start pose,
configuration, full GT trajectory, commands and summary metrics. `/route/status`
publishes JSON with route ID, waypoint, elapsed simulation time, errors and state
(WAITING, ROTATING, TRANSLATING, COMPLETED, ABORTED).

## Control and limits

Stop-and-turn waypoint control uses bounded command acceleration and decelerates
on approach. After final yaw correction, position is checked again; if needed,
the same final waypoint is approached again and final yaw is restored. Completion
requires both tolerances in the same GT sample. Corrective translations target
10% of the position tolerance to leave margin for rotation-induced drift; ordinary
waypoint approaches and the final acceptance tolerances are unchanged. All final corrections share one
`waypoint_timeout` starting at the first final yaw phase; retries never extend it.

| Parameter | Default |
|---|---:|
| max_linear_velocity | 0.10 m/s |
| max_angular_velocity | 0.30 rad/s |
| max_linear_acceleration | 0.10 m/s² |
| max_angular_acceleration | 0.30 rad/s² |
| position_tolerance | 0.015 m |
| heading_tolerance | 0.015 rad |
| waypoint_timeout | 90 simulation s |
| max_tracking_error | 0.20 m |
| feedback_timeout | 0.50 simulation s |

Tracking error is distance to the active finite nominal segment. Tracking-limit
violations, timeout, invalid frame/feedback, time resets and feedback loss abort;
a wall watchdog also detects two seconds without feedback after starting.
Safety aborts command zero immediately. There is no obstacle avoidance.

## Validation

```bash
cd ~/slam_testing
colcon build --packages-select slam_eval_route_controller --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/slam_eval_route_controller --output-on-failure
```

The local harness `scripts/run_validation.py OUTPUT_DIRECTORY --runs 3` uses a
fresh simulator process per trial (with TurtleBot3 and evaluation overlays sourced).
`scripts/analyze_validation.py OUTPUT_DIRECTORY` compares saved trajectories.
Detailed protocols, measurements and limitations are in [VALIDATION.md](VALIDATION.md).
No rosbag or SLAM process is started by these scripts.
