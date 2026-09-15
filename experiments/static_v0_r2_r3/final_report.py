"""Summarize preserved route gates and completed cohort reports."""
import json,hashlib
from pathlib import Path
W=Path('/home/roam5170/slam_testing');OUT=W/'results/static/world_v0/r2_r3_end_to_end';state=json.loads((OUT/'status.json').read_text());gate=json.loads((W/'results/static/world_v0/route_validation_r2_r3/gate.json').read_text());report={'execution':state,'route_validation':{},'cohorts':{},'deviations':['Dataset paths/labels/counts/durations/hashes parameterized; algorithms unchanged.','Route validation wall supervisor 600 s instead of R1 240 s; all simulation-time controller deadlines unchanged.','Initial stationarity uses approved >=5s/<=5mm, not superseded provisional 1mm.','Replay uses accepted message-count/header checks and validated clock guard; raw CDR reserialization hashes remain diagnostic only.','Clearance is GT geometric screening with 0.20m envelope, not physics-contact instrumentation.']}
for route in ['r2','r3']:
 p=W/f'results/static/world_v0/route_validation_r2_r3/{route}/summary.json'
 if p.exists():report['route_validation'][route]=json.loads(p.read_text())
 root=W/f'results/static/world_v0/{route}';entry={}
 for k,f in [('pose_system','baseline_run_001_010/aggregate_report.json'),('map','map_evaluation_001_010/report.json')]:
  p=root/f
  if p.exists():entry[k]=json.loads(p.read_text())
 ref=W/f'reference_maps/static/world_v0/{route}'
 for k,f in [('reference','generation_metadata.json'),('determinism','determinism.json')]:
  if (ref/f).exists():entry[k]=json.loads((ref/f).read_text())
 report['cohorts'][route]=entry
(OUT/'final_report.json').write_text(json.dumps(report,indent=2));print(json.dumps({'status':state['status'],'routes':state['routes']},indent=2))
