import os,signal,subprocess,time,json
from pathlib import Path
p=Path('/tmp/v0_reference_probe');env=os.environ.copy();env['GZ_PARTITION']='v0_r1_reference_inspection';env['GZ_SIM_RESOURCE_PATH']='/home/roam5170/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models'
world='/home/roam5170/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/turtlebot3_world.world'
f=(p/'server.log').open('w');proc=subprocess.Popen(['gz','sim','-s','--headless-rendering','-v','3',world],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
try:
 time.sleep(6)
 report={'process_returncode_before_stop':proc.poll(),'command':['gz','sim','-s','--headless-rendering','-v','3',world]}
 for name,args in [('services',['gz','service','-l']),('set_pose',['gz','service','-i','-s','/world/default/set_pose']),('control',['gz','service','-i','-s','/world/default/control']),('scene',['gz','service','-i','-s','/world/default/scene/info'])]:
  r=subprocess.run(args,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10);report[name]=r.stdout
 (p/'services.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 proc.wait(timeout=15);f.close()
