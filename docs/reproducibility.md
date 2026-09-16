# Reproducibility and repository layout

## Frozen benchmark identity

The [static report](../reports/static_benchmark/README.md) and its
[source manifest](../reports/static_benchmark/data/source_manifest.json) identify
routes, valid cohorts and portable aggregate snapshots; hashes are listed below.
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

## Static benchmark provenance

- Frozen SLAM methodology: `8212a45`; R2/R3 adapters/routes and replacement tooling: `d8cf74b`. Reference core: `09a8aea`; map scoring: `bad6a25`. R1 bag provenance commit: `f93de0e`. Full commits and source hashes are retained in snapshots/manifests.
- SLAM configuration SHA-256: `7a9930fd1e5fea1e798c6cea1e2fe08827cd59b58d4a5902997204f91ac8f937`. rosbag2 `0.26.11`; ROS CLI `0.32.10`.
- Reference geometry: 0.05 m grid; 360 rays spanning 0–6.28 rad; range 0.12–3.5 m; zero added lidar noise. Full GT orientation composes the physical SDF lidar offset `(−0.032, 0, 0.171)` m. The ROS `base_scan` z offset is 0.182 m: the recorded **11 mm discrepancy** is intentionally preserved.
- R1 uses 854/855 reference scans (49 exact, 805 interpolated, first scan omitted without a GT bracket); R2 uses 1,052/1,052 (69 exact, 983 interpolated); R3 uses 1,022/1,022 (59 exact, 963 interpolated). No extrapolation. Each reference was generated twice with identical occupancy hashes.

### Canonical MCAP and reference occupancy SHA-256

| Route | Canonical MCAP | Reference `occupancy.npy` |
|---|---|---|
| R1 | `f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715` | `e6bb2022935e23aa08002deddd5a319249e01d6d13b3ae4fcaeb18380d215272` |
| R2 | `f4fd9096fea5db7c2d4ae90467b176679531ca222e71904e08c39fec4c79ecc8` | `8953777fb7263d6ff8c8ade590684b2666fab2e452f482152ee7ec55c37bb678` |
| R3 | `28a4014cebf25e945384925424fb4d751383195cdeadd445caeb4f950b28402a` | `b2a2328c1b07fe03e160dbde137372e6d05a82233d915021af73bb33d46b39ba` |

### R3 exclusion evidence

Original R3 `run_010` passed the old artifact checks despite 46.79% coverage.
A separate player started approximately 1.6 s before the first logged drop and
overlapped the run. Concurrent playback is confirmed; direct foreign TF receipt
is strongly supported but was not recorded. The user adjudicated it invalid;
original success fields and evidence remain unchanged. Exactly one authorized
replacement (`run_011`) had no other player at preflight and only its intended
player in ongoing process observations. A sole `/clock` authority does not rule
out an external `/tf` publisher. See the [exclusion audit](../reports/static_benchmark/data/r3_exclusion_audit.json),
[replacement audit](../reports/static_benchmark/data/r3_replacement_audit.json) and
[valid cohort](../reports/static_benchmark/data/r3_valid_cohort.json).
