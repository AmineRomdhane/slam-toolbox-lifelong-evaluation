# slam_eval_route_controller

Node `route_controller`: YAML route control from `/ground_truth/pose`, publishing
`geometry_msgs/TwistStamped` on `/cmd_vel` and JSON state on `/route/status`.
No scan, odometry, TF or SLAM feedback. Stop-and-turn motion uses bounded acceleration;
final correction requires position and yaw within tolerance under one shared timeout.

Authoritative [routes](../../benchmark/routes/r1.yaml) live in `benchmark/routes/`.
CMake installs them into the package share directory; `route_file` selects a YAML.
The default remains installed `routes/r1.yaml`. Installed files are build artifacts,
not independent configuration sources. Build this package within this repository.

```bash
ros2 run slam_eval_route_controller route_controller --ros-args \
  -p use_sim_time:=true -p output_dir:=/tmp/unique_route_trial
```

Defaults: linear/angular limits 0.10 m/s and 0.30 rad/s; accelerations 0.10 m/s² and
0.30 rad/s²; position/yaw tolerances 0.015 m/rad; waypoint/final-correction timeout
90 simulation seconds; tracking-error limit 0.20 m; feedback timeout 0.50 simulation
seconds and 2-second wall watchdog. Abort commands zero immediately; no obstacle avoidance.
The output directory must be new and retains actual start pose, trajectory and status.

See [build/test](../../docs/reproducibility.md),
[validation details](../../docs/validation/route_controller.md), and the
[fresh-world validation harness](../../benchmark/tools/validation/run_validation.py).
