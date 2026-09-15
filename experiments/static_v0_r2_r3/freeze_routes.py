"""Freeze exact validated YAMLs and verify inherited control/method source fingerprints."""
import hashlib,json,subprocess,shutil
from pathlib import Path
W=Path('/home/roam5170/slam_testing');HERE=W/'experiments/static_v0_r2_r3';P=W/'src/slam_eval_route_controller';gate=json.loads((W/'results/static/world_v0/route_validation_r2_r3/gate.json').read_text());assert gate['status']=='PASSED'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
base=json.loads((W/'bags/static/world_v0/r1/evidence/route_run/configuration.json').read_text())['parameters'];report={'routes':{},'controller_parameters':base,'route_validation_runs_each':3,'methodology_sources':{},'base_slam_commit':'8212a45','base_reference_commit':'09a8aea','base_map_evaluator_commit':'bad6a25'}
for route in ['r2','r3']:
 root=W/f'results/static/world_v0/route_validation_r2_r3/{route}';s=json.loads((root/'summary.json').read_text());assert len(s['runs'])==3
 for i,r in enumerate(s['runs'],1):
  assert r['state']=='COMPLETED' and r['final_position_error']<=.015 and r['final_yaw_error']<=.015 and r['continuous_clearance_lower_bound_m']>0 and not r['geometric_collision']
  assert json.loads((root/f'run_{i}/configuration.json').read_text())['parameters']==base
 report['routes'][route]={'sha256':sha(P/f'routes/{route}.yaml'),'validation_summary_sha256':sha(root/'summary.json'),'validation_summary':str(root/'summary.json')}
for name,h in gate['source_hashes'].items():assert sha(P/name)==h
for directory,commit in [('experiments/static_v0_r1','8212a45'),('experiments/reference_v0_r1','09a8aea'),('experiments/map_evaluation_v0_r1','bad6a25')]:
 for p in (W/directory).iterdir():
  if not p.is_file():continue
  relative=str(p.relative_to(W));assert p.read_bytes()==subprocess.check_output(['git','-C',str(W),'show',commit+':'+relative]);report['methodology_sources'][relative]=sha(p)
(HERE/'freeze_manifest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['routes'],indent=2))
