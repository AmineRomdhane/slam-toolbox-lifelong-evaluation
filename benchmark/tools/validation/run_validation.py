"""Fresh-process resets avoid Gazebo 8.11 reset/respawn rendering-state reuse."""
import subprocess,time,json,pathlib,os,signal,argparse
parser=argparse.ArgumentParser()
parser.add_argument('output_directory',type=pathlib.Path)
parser.add_argument('--runs',type=int,default=5)
args=parser.parse_args()
WS=pathlib.Path(__file__).resolve().parents[3]
root=args.output_directory
root.mkdir(parents=True,exist_ok=False)
def start(args,log):
    return subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def stop(p):
    if p is None:return
    try:os.killpg(p.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
for i in range(1,args.runs+1):
    print('FRESH WORLD RUN',i,flush=True)
    sim=gt=ctl=None
    try:
        with (root/f'simulator_{i}.log').open('w') as sl,(root/f'ground_truth_{i}.log').open('w') as gl,(root/f'run_{i}.log').open('w') as cl:
            sim=start(['ros2','launch','turtlebot3_gazebo','turtlebot3_world.launch.py','use_sim_time:=true'],sl)
            time.sleep(8)
            if sim.poll() is not None:raise RuntimeError('Simulator failed at startup')
            gt=start([str(WS/'install/slam_eval_ground_truth/lib/slam_eval_ground_truth/ground_truth_node')],gl)
            time.sleep(4)
            out=root/f'run_{i}'
            ctl=start([str(WS/'install/slam_eval_route_controller/lib/slam_eval_route_controller/route_controller'),'--ros-args','-p','use_sim_time:=true','-p',f'output_dir:={out}'],cl)
            ctl.wait(timeout=240)
            if ctl.returncode:raise RuntimeError(f'Controller exited {ctl.returncode}')
            result=json.loads((out/'result.json').read_text())
            print('RUN',i,json.dumps(result),flush=True)
            if result['state']!='COMPLETED':raise RuntimeError('Route aborted')
    finally:
        stop(ctl);stop(gt);stop(sim)
    time.sleep(3)
