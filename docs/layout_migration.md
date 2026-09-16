# Publishable layout migration

Base commit: `d8cf74bcbb1413ec7a315092ef8e817d83e81d6e`. Frozen raw data and report values are not moved or rewritten.

## Moves (git mv)

| Previous path | Published path |
|---|---|
| `experiments/static_v0_r1/clock_guard.py` | `benchmark/tools/slam/clock_guard.py` |
| `experiments/static_v0_r1/evaluate.py` | `benchmark/tools/slam/evaluate.py` |
| `experiments/static_v0_r1/run_pilot.py` | `benchmark/tools/slam/run_pilot.py` |
| `experiments/static_v0_r1/mapper_params_online_async.yaml` | `benchmark/config/mapper_params_online_async.yaml` |
| `experiments/static_v0_r1/configuration_manifest.json` | `benchmark/manifests/configuration_manifest.json` |
| `experiments/static_v0_r1/METHODOLOGY.md` | `docs/methodology.md` |
| `experiments/static_v0_r1/test_clock_guard.py` | `tests/test_clock_guard.py` |
| `experiments/static_v0_r1/test_evaluation.py` | `tests/test_evaluation.py` |
| `experiments/reference_v0_r1/CMakeLists.txt` | `benchmark/tools/reference/CMakeLists.txt` |
| `experiments/reference_v0_r1/cast_reference.cpp` | `benchmark/tools/reference/cast_reference.cpp` |
| `experiments/reference_v0_r1/grid.py` | `benchmark/tools/reference/grid.py` |
| `experiments/reference_v0_r1/prepare.py` | `benchmark/tools/reference/prepare.py` |
| `experiments/reference_v0_r1/run.py` | `benchmark/tools/reference/run.py` |
| `experiments/reference_v0_r1/validate.py` | `benchmark/tools/reference/validate.py` |
| `experiments/reference_v0_r1/METHOD.md` | `docs/reference_maps.md` |
| `experiments/reference_v0_r1/test_reference.py` | `tests/test_reference.py` |
| `experiments/map_evaluation_v0_r1/score.py` | `benchmark/tools/map/score.py` |
| `experiments/map_evaluation_v0_r1/test_score.py` | `tests/test_score.py` |
| `experiments/static_v0_r2_r3/freeze_manifest.json` | `benchmark/manifests/r2_r3_freeze_manifest.json` |
| `reference_maps/static/world_v0/r1/decisions.json` | `benchmark/references/world_v0/decisions.json` |
| `reference_maps/static/world_v0/r1/input_inspection.json` | `benchmark/references/world_v0/input_inspection.json` |
| `reference_maps/static/world_v0/r1/services.json` | `benchmark/references/world_v0/services.json` |
| `src/slam_eval_ground_truth/verification/runtime_results.json` | `benchmark/manifests/validation/ground_truth/runtime_results.json` |
| `src/slam_eval_ground_truth/verification/shutdown.txt` | `benchmark/manifests/validation/ground_truth/shutdown.txt` |
| `src/slam_eval_ground_truth/scripts/verify_runtime.py` | `benchmark/tools/validation/verify_ground_truth.py` |
| `src/slam_eval_route_controller/scripts/analyze_validation.py` | `benchmark/tools/validation/analyze_validation.py` |
| `src/slam_eval_route_controller/scripts/run_validation.py` | `benchmark/tools/validation/run_validation.py` |
| `src/slam_eval_route_controller/VALIDATION.md` | `docs/validation/route_controller.md` |
| `src/slam_eval_route_controller/test/test_core.py` | `tests/test_controller.py` |
| `src/slam_eval_route_controller/routes/r1.yaml` | `benchmark/routes/r1.yaml` |
| `src/slam_eval_route_controller/routes/r2.yaml` | `benchmark/routes/r2.yaml` |
| `src/slam_eval_route_controller/routes/r3.yaml` | `benchmark/routes/r3.yaml` |

## Removed tracked files

Completed one-shot scripts/probes and redundant READMEs are retained in Git history. Local reference ignore rules are consolidated in root `.gitignore`.

- `experiments/map_evaluation_v0_r1/README.md`
- `experiments/reference_v0_r1/README.md`
- `experiments/static_v0_r1/README.md`
- `experiments/static_v0_r2_r3/README.md`
- `experiments/static_v0_r2_r3/adapt_step.py`
- `experiments/static_v0_r2_r3/analyze_route.py`
- `experiments/static_v0_r2_r3/check_adapters.py`
- `experiments/static_v0_r2_r3/end_to_end.py`
- `experiments/static_v0_r2_r3/final_report.py`
- `experiments/static_v0_r2_r3/freeze_routes.py`
- `experiments/static_v0_r2_r3/reference_route.py`
- `experiments/static_v0_r2_r3/validate_routes.py`
- `reference_maps/static/world_v0/.gitignore`
- `reference_maps/static/world_v0/r1/.gitignore`
- `reference_maps/static/world_v0/r1/CMakeLists.txt`
- `reference_maps/static/world_v0/r1/README.md`
- `reference_maps/static/world_v0/r1/inspect.cpp`
- `reference_maps/static/world_v0/r1/inspect_inputs.py`
- `reference_maps/static/world_v0/r1/probe.py`

## Other changes

- Root/package landing pages rewritten; methodology split into docs/methodology.md, docs/metrics.md and docs/reproducibility.md.
- Report source links updated; JSON snapshots, numbers and figure bytes preserved. Existing untracked report files are included in this publication.
- Test imports and package CMake paths updated. Routes are installed from benchmark/routes; no runtime/controller limit changes.
- Tool workspace roots now derive from script location; reference external asset locations use Path.home(). Frozen asset metadata remains unchanged.
- SLAM runner snapshots now include relocated config/manifest explicitly. Numerical algorithms and public ROS interfaces are unchanged.
- Map scorer source-integrity guard now checks the relocated source manifest instead of comparing current paths against pre-migration Git paths; reference occupancy/commit checks remain.
- Completed R2/R3 orchestration/probe entry points are retired, not silently generalized. Historical exact execution is available from the recorded frozen commit.
- Generated reference outputs remain ignored at their original paths; no experiment is run for migration.

## Verification

[Verification results](layout_verification.json): 46 offline tests; both ROS package test entries; reference caster compiled; 1,773 protected raw files unchanged; route/config bytes and all six report aggregate snapshots preserved. No experiments executed.

The controller test now guards `unittest.main()` for discovery; assertions are unchanged.
Empty source directories and these obsolete ignored Python caches were removed:

- `experiments/reference_v0_r1/__pycache__/grid.cpython-312.pyc`
- `experiments/reference_v0_r1/__pycache__/validate.cpython-312.pyc`
- `experiments/reference_v0_r1/__pycache__/prepare.cpython-312.pyc`
- `experiments/map_evaluation_v0_r1/__pycache__/test_score.cpython-312.pyc`
- `experiments/map_evaluation_v0_r1/__pycache__/score.cpython-312.pyc`
- `experiments/static_v0_r1/__pycache__/clock_guard.cpython-312.pyc`
- `experiments/static_v0_r1/__pycache__/run_pilot.cpython-312.pyc`
- `experiments/static_v0_r1/__pycache__/evaluate.cpython-312.pyc`
