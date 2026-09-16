# Ideal-observed reference maps

The counts in the following protocol describe R1; R2/R3 association counts and
reference hashes are in the [static report](../reports/static_benchmark/README.md).

Input is only canonical `/scan` timestamps/geometry and `/ground_truth/pose`.
The first scan at 11.000 s is explicitly omitted: GT starts at 11.008 s.
49 exact plus 805 bracketed poses give 854 scans and 307,440 rays. No extrapolation.
Use integer-nanosecond brackets, linear XYZ, normalized shortest-arc quaternion
SLERP. Compose full SE(3) with physical SDF offset (-0.032,0,0.171) m and identity
rotation. Recorded ROS base_scan is 11 mm higher; it is not substituted.

World geometry and mesh assets are unmodified, checked against inspection hashes.
Unversioned Fuel includes resolve explicitly to cached Ground Plane v5/Sun v3;
canonical metadata lacked their hashes, so historical identity cannot be proven.
No downloads. Gazebo Sim 8.11.0 Sensors/Ogre2 retains physical SDF angular and
range geometry: 360, [0,6.28], range [0.12,3.5] m. Physical angle step is 6.28/359;
ROS reports its float32 representation. Noise mean/stddev are zero; normal sigma
is .01 m. Offline update rate 100 Hz only controls waiting, not geometry or poses.

Each pose creates a new sensor under an identity static carrier, on a unique
frame name; Gazebo's scan world_pose equals the physical world pose in this setup.
Accept after two frames matching requested position within 1e-7 m and quaternion
absolute-dot error <=1e-9. Remove the carrier after sampling. Thus no previous
sensor's queued scan can be assigned to a new pose. Gazebo lidar's native GPU
rendering/facet/float precision is retained; this is not an analytic mesh solver.

Grid resolution .05 m, origin snapped below all lidar XY positions minus 3.5 m,
with one padding cell; matching upper bound. Use full 3D rays, project segments
and endpoints to XY. Amanatides-Woo traversal with half-open cells; simultaneous
corner crossings advance both axes. Finite range <3.5 gives a genuine occupied
endpoint cell, preceding cells free. Infinite or exactly 3.5 ranges are no hit
before maximum; traversed cells through endpoint free. No angular gap filling,
no traversal after hit. Per-cell first-hit evidence takes precedence over free
traversal evidence; preserve both counts and first-hit ray index. Origin-to-min
range is treated free per the requested rule, including the physical blind zone.

Store int8 occupancy.npy [y,x], row zero at minimum y, with -1/0/100 values.
PGM rows are vertically flipped, pixels 205/254/0; YAML trinary thresholds.
The authoritative deterministic hash is the complete uncompressed occupancy.npy
file. Keep rays, poses, associations, free/hit counts, and first-hit witnesses.

Validation checks nine occluded post center cells, observed near-route post
surfaces, all six enclosing body wall faces, and unknown cells outside its outer
perimeter. Known wall coordinates come from original mesh vertices for diagnostics
only. No custom intersections are used to generate ranges. Unknown regions can
be filled only by direct ray traversal or a first hit. Geometry behind a single
hit may be observed legitimately at other scan poses. The resulting references are scored offline using the method in [metrics.md](metrics.md).
