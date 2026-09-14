# slam_eval_ground_truth

Independent trajectory ground truth from the running Gazebo world. No simulator
plugins, odometry subscriptions, TF lookups, or TF broadcasts are used.

## Build and run

Use a clean shell sourced only from /opt/ros/jazzy/setup.bash:

```bash
cd ~/slam_testing
colcon build --packages-select slam_eval_ground_truth --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/slam_eval_ground_truth --output-on-failure
source install/local_setup.bash
ros2 run slam_eval_ground_truth ground_truth_node
```

The installed vendor configs were inspected before writing CMake. This Jazzy
package links gz-transport13::gz-transport13, gz-msgs10::gz-msgs10 and
gz-math7::gz-math7. No additional dependencies were installed.

| Startup parameter | Default |
|---|---|
| world_name | default |
| model_name | burger |
| tracked_frame | base_footprint |
| output_frame | gazebo_world |
| output_topic | /ground_truth/pose |

Parameters are read at startup. Restart to change them. The maintainer email in
package.xml is a placeholder to replace before publishing this package.

## Frame and time contract

The ROS output is geometry_msgs/msg/PoseStamped: full XYZ and quaternion of
tracked_frame, expressed in the Gazebo world's coordinates. output_frame is an
explicit label for that reference (default gazebo_world); it does not establish
a transform to ROS map or odom. PoseStamped has no child_frame_id; the tracked
body frame is specified by tracked_frame.

Resolve a unique top-level model and direct child link through
/world/<world_name>/scene/info. Match their names AND IDs in each
/world/<world_name>/dynamic_pose/info Pose_V. Compute:
T_world_tracked = T_world_model * T_model_tracked.
Nested models and ambiguous names are deliberately unsupported. Both poses must
exist in the same input message. No stored pose is reused.

Copy Pose_V.header.stamp sec/nsec exactly. Reject invalid stamps/poses and
suppress duplicate output stamps. A backward input stamp begins a new epoch,
invalidates entity resolution, and permits the new simulation time after
rediscovery; timestamps are monotonic within an epoch, not across world resets.
No wall-time stamp or /clock subscription is needed by the extractor.

Poll scene identity every 500 ms (200 ms service timeout), and subscribe to
scene/deletion. Missing poses, deleted entities, or resets invalidate identity.
Discard discovery replies spanning an invalidation. Resolve IDs again after
replacement/reset and log each resolution. If the stream stops, publish nothing;
there is no timer-driven extrapolation. A paused world may repeat timestamps;
these are suppressed.

Gazebo transport callbacks and discovery use a mutex. Publisher QoS is reliable,
volatile, keep-last 100. The node reports received, published,
missing_entity_messages, time_resets, entity_id_changes, invalid_messages and
duplicate_stamps on normal SIGINT/SIGTERM shutdown. The ID-change counter counts
changes of the resolved (model, link) pair, not initial discovery or an unchanged
pair after reset. Missing-message counts include initial unresolved messages.

## Verification

```bash
python3 src/slam_eval_ground_truth/scripts/verify_runtime.py
```

The read-only verifier observes 35 seconds, discards 5 seconds of warm-up, and
measures 30 seconds. It also captures native Gazebo JSON messages and ROS /clock
to check exact timestamps, not just similar numerical time ranges. It creates no
routes, commands, or bags. It currently targets the default world/output topic.

See verification/runtime_results.json for this run. 1,759 poses were received at
58.619925 wall Hz / 58.629315 simulation Hz. All 1,759 timestamps exactly matched
both captured Gazebo stamps and ROS clock stamps; none were non-increasing.
Stationary robot pose was approximately (-2.014231723, -0.5, 0.010008797),
quaternion XYZW (-3.491e-9, -0.006229758, -1.391e-8, 0.999980595).
Resolved IDs: model burger=43, tracked link base_footprint=44.

The extractor was stopped after verification; shutdown counters are saved beside
the results. ROS graph inspection found no /odom or TF endpoints on this node.
The focused C++ test exercises non-identity rotated composition, exact timestamp
copying, reordered entities, duplicate suppression, reset, replacement, invalid
quaternion rejection, and ambiguous scene rejection. Actual world reset/removal
was not performed: the existing simulation was left unchanged.

For the current upstream Burger SDF, model-to-base_footprint is identity.
The SDF and ROS URDF have differing link-frame geometry; the extractor follows
actual Gazebo state, not URDF joint origins. Ground truth means simulator state,
not a claim that its physics/sensors reproduce a real robot perfectly.
