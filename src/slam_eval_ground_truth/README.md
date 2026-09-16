# slam_eval_ground_truth

ROS 2 C++ node `ground_truth_node`. Subscribes directly to Gazebo Transport
`/world/<world_name>/dynamic_pose/info`, resolves scene entity IDs and publishes
`geometry_msgs/PoseStamped` on `/ground_truth/pose` with the original Gazebo timestamp.
Computes `T_world_model * T_model_tracked`, retaining full XYZ/quaternion.
No odometry subscription, TF lookup or TF broadcast.

Defaults: `world_name=default`, `model_name=burger`, `tracked_frame=base_footprint`,
`output_frame=gazebo_world`, `output_topic=/ground_truth/pose`.
Reset/disappearance/replacement handling and shutdown counters are implemented.

```bash
ros2 run slam_eval_ground_truth ground_truth_node
```

Requires Jazzy rclcpp and Gazebo Transport 13 / Messages 10 / Math 7 vendor packages.
See [build/test instructions](../../docs/reproducibility.md),
[runtime verifier](../../benchmark/tools/validation/verify_ground_truth.py), and
[recorded verification](../../benchmark/manifests/validation/ground_truth/runtime_results.json).
GT is evaluation-only for SLAM, and feedback for the separate route controller.
