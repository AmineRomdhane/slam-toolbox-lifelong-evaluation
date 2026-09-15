# V0/R1 ideal observed reference

Offline reference generation from canonical GT poses and Gazebo's noise-free GPU
lidar. No SLAM, odometry, robot controller, ROS bridge, or map scoring is used.

Build with CMake after sourcing `/opt/ros/jazzy/setup.bash`. Run
`python3 run.py <new-output-directory> --caster <build>/cast_reference`.
Each invocation starts a fresh isolated headless Gazebo server. Output directories
must not exist. Run twice and compare `occupancy.npy` SHA-256 before acceptance.
`python3 -m unittest -v test_reference` runs traversal/interpolation tests.
See METHOD.md and output JSON reports for methodology and validation.
