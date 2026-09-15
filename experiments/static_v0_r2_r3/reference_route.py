"""Use unchanged R1 ray casting/interpolation/grid/validation, with dataset-derived counts."""
import argparse,bisect,hashlib,json,os,signal,subprocess,sys,time,shutil
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
W=Path('/home/roam5170/slam_testing');RBASE=W/'experiments/reference_v0_r1';sys.path.insert(0,str(RBASE));from grid import make_grid
p=argparse.ArgumentParser();p.add_argument('route',choices=['r2','r3']);p.add_argument('generation',type=int);args=p.parse_args();route=args.route;ROOT=W/f'reference_maps/static/world_v0/{route}';out=ROOT/f'generation_{args.generation:03d}';BAG=W/f'bags/static/world_v0/{route}/v0_{route}_canonical'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
import yaml
meta=yaml.safe_load((BAG.parent/'metadata.yaml').read_text());assert meta['canonical_status']=='ACCEPTED';assert sha(BAG/f'v0_{route}_canonical_0.mcap')==meta['canonical_bag_sha256'];assert not out.exists()
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(BAG),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan','/ground_truth/pose']));scans=[];gt=[]
while reader.has_next():
 topic,data,_=reader.read_next();m=deserialize_message(data,LaserScan if topic=='/scan' else PoseStamped);(scans if topic=='/scan' else gt).append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
reader=None
outside=[t for t in scans if t<gt[0] or t>gt[-1]];assert outside in [[],[scans[0]]],'Unexpected GT coverage gap';exact=len(set(scans)&set(gt));counts={'exact':exact,'interpolated':len(scans)-exact-len(outside),'omitted_no_bracket':len(outside)};n=len(scans)-len(outside)
s=(RBASE/'prepare.py').read_text();s=s.replace('bags/static/world_v0/r1/v0_r1_canonical',f'bags/static/world_v0/{route}/v0_{route}_canonical').replace("bag/'v0_r1_canonical_0.mcap'",f"bag/'v0_{route}_canonical_0.mcap'").replace('f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715',meta['canonical_bag_sha256']);s=s.replace('len(scans)==855 and len(gt)==10041',f'len(scans)=={len(scans)} and len(gt)=={len(gt)}');s=s.replace("{'exact':49,'interpolated':805,'omitted_no_bracket':1}",repr(counts));s=s.replace("assert associations[0]['stamp_ns']==11000000000 and associations[0]['method']=='omitted_no_bracket'",f"assert [a['stamp_ns'] for a in associations if a['method']=='omitted_no_bracket']=={outside!r}");s=s.replace("'scan_count':854,'omitted':1,'rays':854*360",f"'scan_count':{n},'omitted':{len(outside)},'rays':{n}*360")
namespace={'__name__':'dataset_prepare','__file__':str(RBASE/'prepare.py')};exec(compile(s,'dataset_prepare','exec'),namespace);namespace['prepare'](out);(out/'prepare_executed.py').write_text(s)
vs=(RBASE/'validate.py').read_text().replace("{'exact':49,'interpolated':805,'omitted_no_bracket':1}",repr(counts)).replace('len(l)==854',f'len(l)=={n}').replace('len(r)==307440',f'len(r)=={n*360}').replace('V0 / R1','V0 / '+route.upper()).replace('854 scans',f'{n} scans');vn={'__name__':'dataset_validate','__file__':str(RBASE/'validate.py')};exec(compile(vs,'dataset_validate','exec'),vn);(out/'validate_executed.py').write_text(vs)
env=os.environ.copy();env['GZ_PARTITION']=f'reference_{route}_{os.getpid()}';env['GZ_SIM_RESOURCE_PATH']='/home/roam5170/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models'
result={'generator_git_commit':subprocess.check_output(['git','-C',str(W),'rev-parse','HEAD'],text=True).strip(),'base_reference_generator_commit':'09a8aeac574270b6929a9dea760384dd7f3532b1','canonical_bag_sha256':meta['canonical_bag_sha256'],'counts':counts,'scan_count':n,'success':False,'frozen_sources':{p.name:sha(p) for p in RBASE.iterdir() if p.is_file()},'adapter_sha256':sha(Path(__file__))}
try:
 with (out/'gazebo.log').open('w') as f:
  proc=subprocess.Popen(['/tmp/reference_build/cast_reference',str(out/'reference_world.sdf'),str(out/'lidar_poses.csv'),str(out/'rays.csv')],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  try:rc=proc.wait(timeout=3600)
  except BaseException:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=30);raise
  assert rc==0
 result['grid']=make_grid(out,n);result['validation']=vn['validate'](out);result['success']=result['validation']['passed'];assert result['success'],'Reference validation failed'
finally:(out/'generation_metadata.json').write_text(json.dumps(result,indent=2))
if args.generation==2:
 first=ROOT/'generation_001';comparison={}
 for f in ['occupancy.npy','ideal_observed_map.pgm','ideal_observed_map.yaml','rays.csv','free_observations.npy','hit_observations.npy','first_hit_ray.npy','ray_endpoints_xyz.npy','lidar_poses.csv','robot_poses.csv','canonical_gt.csv','interpolation.json']:
  comparison[f]=[sha(first/f),sha(out/f)];assert comparison[f][0]==comparison[f][1],f+' not deterministic'
 for f in out.iterdir():
  if f.is_file() and f.suffix not in ['.log','.py']:shutil.copy2(f,ROOT/f.name)
 result.update(reference_status='VALIDATED_READY_TO_FREEZE',reference_map_sha256=sha(out/'occupancy.npy'),deterministic=True);(ROOT/'generation_metadata.json').write_text(json.dumps(result,indent=2));(ROOT/'determinism.json').write_text(json.dumps({'passed':True,'hashes':comparison},indent=2))
print(json.dumps({'route':route,'generation':args.generation,'counts':counts,'grid':result['grid'],'success':result['success']},indent=2))
