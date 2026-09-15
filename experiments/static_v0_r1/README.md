# V0/R1 async SLAM experiment tooling

Single-run SLAM Toolbox 2.8.5 runner with frozen configuration, resource phases,
rigid SE(2) trajectory evaluation, and explicit success/failure metadata.

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/slam_testing/experiments/static_v0_r1/test_evaluation.py
python3 ~/slam_testing/experiments/static_v0_r1/run_pilot.py --run-id run_002
```

Existing output directories, changed input/configuration or conflicting publishers
are refused. There is no repeat loop or automatic retry. Results and raw samples
are git-ignored under results/static/world_v0/r1/<run-id>/; source snapshots and
hashes are retained per run. Ground truth is evaluation-only.

The configuration changes only simulation time and explicit 3.5 m laser range
from installed defaults. See METHODOLOGY.md for exact scan terminology, alignment,
association, RPE, phase windows and failure handling; configuration_manifest.json
records the frozen configuration hash. Detailed results stay in each run directory.
