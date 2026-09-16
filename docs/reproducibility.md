# Reproducibility and repository layout

## Frozen benchmark identity

The [static report](../reports/static_benchmark/README.md) and its
[source manifest](../reports/static_benchmark/data/source_manifest.json) contain
canonical bag/reference hashes, routes, valid cohorts and portable aggregate snapshots.
The current valid R3 cohort is runs 001–009 plus 011; original 010 is invalid/excluded.

Historical commits: SLAM runner/evaluator `8212a45`, reference generator `09a8aea`,
map scorer `bad6a25`, R2/R3 routes/adapters `d8cf74b`. Layout changes do not rewrite
these commits, frozen metadata, result snapshots or their SHA-256 values.
[Historical configuration manifest](../benchmark/manifests/configuration_manifest.json)
and [R2/R3 freeze manifest](../benchmark/manifests/r2_r3_freeze_manifest.json) are
byte-preserved. Paths and file hashes inside historical JSON are provenance at
those commits, not instructions to rewrite current results or pretend relocated
sources have identical hashes. [Layout manifest](../benchmark/manifests/layout.json)
maps every moved/deleted source and pins current tool bytes. The map scorer checks
this current manifest; historical reference identity checks are retained.

## Current authoritative locations

- `benchmark/config/mapper_params_online_async.yaml`: unchanged SLAM YAML.
- `benchmark/routes/r1.yaml`, `r2.yaml`, `r3.yaml`: unchanged route YAMLs; sole tracked authorities.
- `benchmark/references/world_v0/`: byte-preserved inspection/decision metadata.
- `benchmark/tools/slam/`: R1 runner, evaluator, read-only clock guard.
- `benchmark/tools/reference/`: R1 reference preparation/casting/grid/validation.
- `benchmark/tools/map/score.py`: frozen R1 scoring algorithm and cohort defaults.
- `benchmark/tools/validation/`: fresh-world route harness, analysis, GT timing verifier.

The retained execution tools still have their original R1-specific dataset/cohort
assumptions. Relocation does not turn them into generic R2/R3 runners. Completed
one-shot R2/R3 orchestration and exploratory inspection probes were retired from
the publishable tree; exact historical versions remain at `d8cf74b` in Git.
For exact historical execution use a separate checkout of the relevant frozen
commit with the matching original local data paths and dependencies. Never rerun
against an existing output directory or overwrite accepted artifacts.

Raw data is deliberately not relocated: local `bags/`, `results/` and generated
`reference_maps/` remain gitignored. The physical checkout can therefore contain
these extra directories, plus ignored build caches, despite the published layout.
Reference inspection records include absolute asset locations on the original host;
their hashes remain authoritative. TurtleBot3 assets are external dependencies.

## Build and tests

With ROS 2 Jazzy and the recorded Gazebo vendor dependencies installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select slam_eval_ground_truth slam_eval_route_controller --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select slam_eval_ground_truth slam_eval_route_controller --event-handlers console_direct+
colcon test-result --verbose
python3 -m unittest discover -s tests -v
```

`benchmark/COLCON_IGNORE` prevents standalone reference CMake tooling being mistaken
for a ROS package. Route installation reads the single source directory above;
no route/config symlink or duplicate tracked YAML is needed. ROS package runtime
names/topics/default parameters are unchanged. Build only; tests launch no simulator.

The reference caster can separately be compiled using CMake with
`benchmark/tools/reference` as its source directory. Do not run it merely to verify
this layout. Matplotlib/NumPy figure reproduction uses only the report snapshots:

```bash
python3 reports/static_benchmark/render_plots.py
```

## Interpretation and audit

See [methodology](methodology.md), [metrics](metrics.md), and
[reference maps](reference_maps.md). R3 validity and replacement evidence are in
[the audit snapshot](../reports/static_benchmark/data/r3_exclusion_audit.json).
Success fields are execution checks, not an accuracy/coverage acceptance threshold.
This restructuring neither changes those rules nor reruns any experiment.
The [migration ledger](layout_migration.md) lists every tracked move/removal.
