"""Compile emitted adapters without launching ROS or writing dataset artifacts."""
import ast,json,sys,yaml
from pathlib import Path
from types import SimpleNamespace
W=Path('/home/roam5170/slam_testing');path=Path(__file__).resolve().parent/'adapt_step.py';tree=ast.parse(path.read_text());branch=next(x for x in tree.body if isinstance(x,ast.If) and isinstance(x.test,ast.Compare) and ast.unparse(x.test).startswith('args.step'))
def stop_source(source,original,save,argslist=[]):compile(source,str(save),'exec');raise StopIteration
for route in ['r2','r3']:
 for step in ['record','validate_bag','replay','slam']:
  g={'Path':Path,'json':json,'sys':sys,'yaml':yaml,'W':W,'BASE':W/'experiments/static_v0_r1','OLD':W/'bags/static/world_v0/r1/evidence','route':route,'upper':route.upper(),'ROOT':W/f'bags/static/world_v0/{route}','RESULT':W/f'results/static/world_v0/{route}','args':SimpleNamespace(step=step,run_id='run_001')}
  def replace(s,a,b):assert a in s,a;return s.replace(a,b)
  g.update(replace=replace,execute=stop_source,dataset=lambda s,route=route:s.replace('world_v0/r1','world_v0/'+route).replace('v0_r1_canonical','v0_'+route+'_canonical').replace('V0_R1','V0_'+route.upper()).replace('routes/r1.yaml','routes/'+route+'.yaml'),metadata=lambda:dict(canonical_status='ACCEPTED',canonical_bag_sha256='test_hash'),validation=lambda:json.loads((W/'bags/static/world_v0/r1/evidence/bag_validation.json').read_text()))
  try:exec(compile(ast.Module(body=[branch],type_ignores=[]),str(path),'exec'),g)
  except StopIteration:print('COMPILED',route,step)
