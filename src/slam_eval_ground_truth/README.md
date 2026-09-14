# slam_eval_ground_truth

ROS 2 package that publishes the **ground-truth pose of the simulated robot**
directly from Gazebo.

The ground truth is used only to evaluate SLAM results. It is independent from
SLAM Toolbox and does not use `/odom`, TF, or SLAM outputs.

## Package contents

```text
slam_eval_ground_truth/
├── src/
│   └── ground_truth_node.cpp
│       Main node that reads the robot pose from Gazebo and publishes
│       /ground_truth/pose.
│
├── scripts/
│   └── verify_runtime.py
│       Runtime verification of pose rate and timestamps.
│
├── verification/
│   ├── runtime_results.json
│   └── shutdown.txt
│       Results from the ground-truth validation.
│
├── test/
│       Tests transform composition, timestamps, resets, entity replacement,
│       and invalid inputs.
│
├── CMakeLists.txt
└── package.xml
```

## How it works

Gazebo publishes the poses of simulated entities on:

```text
/world/<world_name>/dynamic_pose/info
```

The node identifies the robot model and tracked frame and computes:

```text
T_world_tracked = T_world_model × T_model_tracked
```

For the current TurtleBot3 simulation:

```text
model:         burger
tracked frame: base_footprint
```

The resulting pose is published as:

```text
Topic: /ground_truth/pose
Type:  geometry_msgs/msg/PoseStamped
Frame: gazebo_world
```

The full XYZ position and quaternion are preserved.

The original Gazebo simulation timestamp is also preserved so that the
ground-truth trajectory can later be synchronized with SLAM and sensor data.

## Independence from SLAM

The node does not:

- subscribe to `/odom`
- query TF
- broadcast TF
- use SLAM Toolbox outputs
- control the robot

Therefore `/ground_truth/pose` can be used as an independent reference for
trajectory metrics such as ATE and RPE.

## Parameters

| Parameter | Default |
|---|---|
| `world_name` | `default` |
| `model_name` | `burger` |
| `tracked_frame` | `base_footprint` |
| `output_frame` | `gazebo_world` |
| `output_topic` | `/ground_truth/pose` |

## Build and run

```bash
cd ~/slam_testing

colcon build \
  --packages-select slam_eval_ground_truth \
  --symlink-install

source install/local_setup.bash

ros2 run slam_eval_ground_truth ground_truth_node
```

Check the published pose:

```bash
ros2 topic echo /ground_truth/pose
```

## Verification

The node was validated using TurtleBot3 World on ROS 2 Jazzy and Gazebo Sim 8.

A 30-second runtime test measured approximately:

```text
Ground-truth rate:            58.6 Hz
Non-increasing timestamps:    0
Gazebo timestamp matches:     100%
ROS /clock timestamp matches: 100%
```

Detailed results are stored in:

```text
verification/
```

`/ground_truth/pose` is intended for **evaluation only** and must not be used as
an input to SLAM Toolbox.
