"""Generate one reference using an isolated Gazebo Transport partition, no ROS nodes."""
import argparse,hashlib,json,os,signal,subprocess,time
from pathlib import Path
from prepare import prepare,WS,TB,sha
from grid import make_grid
from validate import validate
p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--caster',required=True);args=p.parse_args();out=Path(args.output).resolve();assert not out.exists(),'Refusing to overwrite an attempt';prepare(out)
meta={'generator_git_commit':subprocess.check_output(['git','-C',str(WS),'rev-parse','HEAD'],text=True).strip(),'generator_source_sha256':{f.name:sha(f) for f in Path(__file__).parent.iterdir() if f.is_file()},'started_unix':time.time(),'success':False,'gazebo_version':'8.11.0','gz_sensors_version':'8.2.2','renderer':'Gazebo Ogre2 GPU lidar, headless EGL','source_note':'Lidar.cc stores Sensor::Pose in world_pose; sensor parent carrier at identity makes this world pose. Unique frame per sensor rejects stale messages.'}
env=os.environ.copy();env['GZ_PARTITION']='reference_v0_r1_'+str(os.getpid());env['GZ_SIM_RESOURCE_PATH']=str(TB/'models');env['MPLCONFIGDIR']='/tmp/reference_mpl'
cmd=[str(Path(args.caster).resolve()),str(out/'reference_world.sdf'),str(out/'lidar_poses.csv'),str(out/'rays.csv')];meta['caster_command']=cmd
try:
 with (out/'gazebo.log').open('w') as f:
  proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:rc=proc.wait(timeout=3600)
  except BaseException:
   os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=30);raise
  assert rc==0,f'Gazebo caster failed: {rc}'
 meta['grid']=make_grid(out);meta['validation']=validate(out);meta['success']=meta['validation']['passed']
except BaseException as e:meta['failure']=str(e);raise
finally:
 meta['ended_unix']=time.time();(out/'generation_metadata.json').write_text(json.dumps(meta,indent=2))
print(json.dumps({'output':str(out),'success':meta['success'],'grid':meta.get('grid')},indent=2))
