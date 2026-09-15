"""R1 fresh-world validation procedure, route selection only; fail-fast gate."""
import ast,hashlib,json,os,signal,subprocess,time,sys
from pathlib import Path
W=Path('/home/roam5170/slam_testing');P=W/'src/slam_eval_route_controller';OUT=W/'results/static/world_v0/route_validation_r2_r3'
# Reuse the exact read-only process guard without executing the SLAM runner.
tree=ast.parse((W/'experiments/static_v0_r1/run_pilot.py').read_text());f=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='conflicting_processes');exec(compile(ast.Module(body=[f],type_ignores=[]),'guard','exec'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stop(p):
 if p is None or p.poll() is not None:return
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
def start(cmd,f):return subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
assert not OUT.exists();OUT.mkdir(parents=True)
gate={'status':'RUNNING','runs_per_route':3,'routes':{},'source_hashes':{n:sha(P/n) for n in ['scripts/control_core.py','scripts/controller.py','scripts/analyze_validation.py','routes/r1.yaml','routes/r2.yaml','routes/r3.yaml']},'clearance_criterion':'Existing conservative geometric collision screening: 0.20 m circular envelope, sampled clearance minus largest GT step must be positive. No new provisional clearance threshold.','supervision_timeout_wall_seconds':600,'supervision_note':'R1 harness wall timeout 240s expanded to 600s for longer routes/more turns; controller waypoint/final-phase deadlines remain 90 simulation seconds.'}
try:
 for route in ['r2','r3']:
  root=OUT/route;root.mkdir();gate['routes'][route]={'attempted':0,'passed':0}
  for i in range(1,4):
   assert not conflicting_processes(),conflicting_processes();sim=gt=ctl=None
   print('START',route,i,flush=True);gate['routes'][route]['attempted']+=1
   try:
    with (root/f'simulator_{i}.log').open('w') as sl,(root/f'ground_truth_{i}.log').open('w') as gl,(root/f'run_{i}.log').open('w') as cl:
     sim=start(['ros2','launch','turtlebot3_gazebo','turtlebot3_world.launch.py','use_sim_time:=true'],sl);time.sleep(8)
     if sim.poll() is not None:raise RuntimeError('Simulator failed at startup')
     gt=start([str(W/'install/slam_eval_ground_truth/lib/slam_eval_ground_truth/ground_truth_node')],gl);time.sleep(4)
     ctl=start([str(W/'install/slam_eval_route_controller/lib/slam_eval_route_controller/route_controller'),'--ros-args','-p','use_sim_time:=true','-p',f'route_file:={P}/routes/{route}.yaml','-p',f'output_dir:={root}/run_{i}'],cl)
     ctl.wait(timeout=600)
     if ctl.returncode:raise RuntimeError(f'Controller exited {ctl.returncode}')
   finally:stop(ctl);stop(gt);stop(sim)
   r=json.loads((root/f'run_{i}/result.json').read_text());subprocess.run([sys.executable,str(P/'scripts/analyze_validation.py'),str(root)],stdout=(root/f'analysis_after_{i}.json').open('w'),check=True)
   a=json.loads((root/'summary.json').read_text())['runs'][-1]
   print('DONE',route,i,json.dumps(a),flush=True)
   if r['state']!='COMPLETED':raise RuntimeError(f'{route} run {i} aborted: '+r['reason'])
   if r['final_position_error']>.015 or r['final_yaw_error']>.015:raise RuntimeError(f'{route} run {i} final tolerance failure')
   if a['geometric_collision'] or a['continuous_clearance_lower_bound_m']<=0:raise RuntimeError(f'{route} run {i} unsafe clearance')
   if r['maximum_tracking_error']>.20:raise RuntimeError(f'{route} run {i} tracking safety violation')
   if a['max_commanded_linear_acceleration']>.100001 or a['max_commanded_angular_acceleration']>.300001:raise RuntimeError(f'{route} run {i} acceleration violation')
   gate['routes'][route]['passed']+=1;(OUT/'gate.json').write_text(json.dumps(gate,indent=2));time.sleep(3)
 gate['status']='PASSED'
except BaseException as e:
 gate['status']='FAILED_STOP';gate['failure']=str(e);print('STOP',str(e),flush=True)
finally:(OUT/'gate.json').write_text(json.dumps(gate,indent=2))
print('VALIDATION_GATE',gate['status'],flush=True)
