# SLAM Toolbox static benchmark

Reproducible evaluation of SLAM Toolbox 2.8.5 on ROS 2 Jazzy, using TurtleBot3
Burger in Gazebo World V0. Three routes, one immutable canonical bag per route,
and ten valid SLAM trials per route establish the static baseline.

**[Read the R1/R2/R3 static benchmark report](reports/static_benchmark/README.md).**
It includes aggregate pose, resource and map metrics, plots and the R3 exclusion audit.

| Directory | Contents |
|---|---|
| [docs](docs/reproducibility.md) | Methodology, metrics, build/test and provenance |
| [benchmark](benchmark/manifests/layout.json) | Authoritative config/routes, manifests, reference metadata and tools |
| [reports](reports/static_benchmark/README.md) | Publishable report, figures, frozen summary snapshots and figure script |
| [src](src/slam_eval_ground_truth/README.md) | ROS packages: independent Gazebo GT and deterministic route controller |
| [tests](tests/test_layout.py) | Offline benchmark, controller and layout tests |

Quick checks from the repository root (Python dependencies: NumPy, SciPy,
Matplotlib, PyYAML):

```bash
python3 -m unittest discover -s tests -v
python3 reports/static_benchmark/render_plots.py
```

ROS package build/test instructions and historical reproduction are documented in
[reproducibility](docs/reproducibility.md). Raw `bags/`, `results/`, generated
`reference_maps/`, and colcon output remain local and gitignored. No dynamic
benchmark results are claimed. This repository contains analysis tooling, not
redistributed TurtleBot3 world assets or ROS/Gazebo dependencies.
