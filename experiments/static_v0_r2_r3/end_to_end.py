"""Two gated static cohorts; immutable inputs, exactly ten SLAM attempts each, no retries."""
import hashlib,json,os,subprocess,sys,time,yaml
from pathlib import Path
W=Path('/home/roam5170/slam_testing');HERE=Path(__file__).resolve().parent;OUT=W/'results/static/world_v0/r2_r3_end_to_end';gate=json.loads((W/'results/static/world_v0/route_validation_r2_r3/gate.json').read_text());assert gate['status']=='PASSED';assert not OUT.exists();OUT.mkdir()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
COMMIT=subprocess.check_output(['git','-C',str(W),'rev-parse','HEAD'],text=True).strip();assert not subprocess.check_output(['git','-C',str(W),'status','--porcelain']).strip()
state={'status':'RUNNING','freeze_commit':COMMIT,'routes':{},'no_retries':True,'methodology':'Unchanged frozen R1 algorithms; dataset-specific paths, labels, counts and measured durations via auditable adapters'}
def save():(OUT/'status.json').write_text(json.dumps(state,indent=2))
def run(script,args,log,allow_failure=False):
 print('START',script,*args,flush=True)
 with Path(log).open('w') as f:r=subprocess.run([sys.executable,str(HERE/script),*args],stdout=f,stderr=subprocess.STDOUT)
 print('END',script,*args,'exit',r.returncode,flush=True)
 if r.returncode and not allow_failure:raise RuntimeError(f'{script} {args} failed; see {log}')
 return r.returncode
try:
 for route in ['r2','r3']:
  entry=state['routes'][route]={'stage':'record','route_sha256':sha(W/f'src/slam_eval_route_controller/routes/{route}.yaml')};save();logs=OUT/route;logs.mkdir()
  # Existing fresh-world recording/GT/settling procedure, then offline and replay validation.
  for step in ['record','metadata','validate_bag','replay']:
   entry['stage']=step;save();run('adapt_step.py',[step,route],logs/(step+'.log'))
  m=yaml.safe_load((W/f'bags/static/world_v0/{route}/metadata.yaml').read_text());entry['bag_sha256']=m['canonical_bag_sha256'];entry['stage']='slam';save()
  root=W/f'results/static/world_v0/{route}';aggregate=root/'baseline_run_001_010';aggregate.mkdir();manifest={'freeze_commit':COMMIT,'runs':[]}
  for i in range(1,11):
   name=f'run_{i:03d}';assert subprocess.check_output(['git','-C',str(W),'rev-parse','HEAD'],text=True).strip()==COMMIT
   assert not subprocess.check_output(['git','-C',str(W),'status','--porcelain']).strip()
   assert sha(W/f'bags/static/world_v0/{route}/v0_{route}_canonical/v0_{route}_canonical_0.mcap')==entry['bag_sha256'];assert sha(W/'experiments/static_v0_r1/mapper_params_online_async.yaml')=='7a9930fd1e5fea1e798c6cea1e2fe08827cd59b58d4a5902997204f91ac8f937'
   rc=run('adapt_step.py',['slam',route,'--run-id',name],logs/(name+'.log'),True);mp=root/name/'run_metadata.json'
   if not mp.exists():
    mp.parent.mkdir(exist_ok=True);mp.write_text(json.dumps({'success':False,'failure_reason':f'Runner exited {rc} before metadata','failure_sim_time':None,'failure_route_fraction':None,'trajectory_coverage':0.},indent=2))
   rm=json.loads(mp.read_text());manifest['runs'].append({'run_id':name,'returncode':rc,'success':rm['success'],'failure_reason':rm.get('failure_reason')});(aggregate/'execution_manifest.json').write_text(json.dumps(manifest,indent=2));entry['slam_attempts']=len(manifest['runs']);entry['slam_successes']=sum(x['success'] for x in manifest['runs']);save();time.sleep(3)
  manifest['completed_attempts']=10;(aggregate/'execution_manifest.json').write_text(json.dumps(manifest,indent=2));run('analyze_route.py',['aggregate',route],logs/'aggregate.log')
  entry['stage']='reference';save()
  for generation in [1,2]:run('reference_route.py',[route,str(generation)],logs/f'reference_{generation}.log')
  ref=json.loads((W/f'reference_maps/static/world_v0/{route}/generation_metadata.json').read_text());entry['reference_sha256']=ref['reference_map_sha256'];entry['reference_counts']=ref['counts'];entry['stage']='map_score';save();run('analyze_route.py',['map_score',route],logs/'map_score.log');entry['stage']='COMPLETE';save()
 state['status']='COMPLETE'
except BaseException as e:state['status']='STOPPED';state['failure']=str(e);print('STOPPED',str(e),flush=True)
finally:save()
print('END_TO_END',state['status'],flush=True)
