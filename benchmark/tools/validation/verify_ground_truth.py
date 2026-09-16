"""Read-only 30-second ROS/Gazebo timing check; no bags or motion commands."""
import json, subprocess, threading, time
import rclpy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

def ns(stamp):
    return int(stamp.sec)*1000000000 + stamp.nanosec

rclpy.init()
node = rclpy.create_node('ground_truth_verifier')
poses, clocks, source_stamps = [], set(), set()
latest_clock = [None]
def on_pose(m):
    poses.append((time.monotonic(), ns(m.header.stamp), m, latest_clock[0]))
def on_clock(m):
    latest_clock[0] = ns(m.clock)
    clocks.add(latest_clock[0])
sub = node.create_subscription(PoseStamped, '/ground_truth/pose', on_pose, 1000)
clock_sub = node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
proc = subprocess.Popen(['gz','topic','-e','-t','/world/default/dynamic_pose/info',
                         '-d','36','--json-output'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
errors=[]
def read_source():
    decoder=json.JSONDecoder()
    pending=''
    while True:
        chunk=proc.stdout.read1(65536)
        if not chunk: break
        pending+=chunk.decode()
        while pending.strip():
            pending=pending.lstrip()
            try: obj,end=decoder.raw_decode(pending)
            except json.JSONDecodeError: break
            pending=pending[end:]
            try:
                s=obj['header']['stamp']
                source_stamps.add(int(s.get('sec',0))*1000000000+int(s.get('nsec',0)))
            except (KeyError,TypeError) as e: errors.append(str(e))
    if pending.strip(): errors.append('Unparsed source JSON: '+pending[:100])
thread=threading.Thread(target=read_source)
thread.start()
start=time.monotonic()
while time.monotonic()-start < 35:
    rclpy.spin_once(node, timeout_sec=0.02)
endpoints={}
for name,nsname in node.get_node_names_and_namespaces():
    if name=='slam_eval_ground_truth':
        endpoints={'publishers':node.get_publisher_names_and_types_by_node(name,nsname),
                   'subscriptions':node.get_subscriber_names_and_types_by_node(name,nsname)}
thread.join(timeout=5)
if thread.is_alive():
    proc.terminate(); thread.join(timeout=2)
values=[p for p in poses if start+5 <= p[0] < start+35]
report={'window_wall_seconds':30,'samples':len(values),'source_parse_errors':errors,
        'extractor_endpoints':endpoints}
if len(values)>1:
    report.update(wall_hz=(len(values)-1)/(values[-1][0]-values[0][0]),
                  stamp_hz=(len(values)-1)*1e9/(values[-1][1]-values[0][1]),
                  non_increasing_stamps=sum(b[1]<=a[1] for a,b in zip(values,values[1:])),
                  exact_gazebo_stamp_matches=sum(p[1] in source_stamps for p in values),
                  exact_ros_clock_stamp_matches=sum(p[1] in clocks for p in values),
                  frame_ids=sorted(set(p[2].header.frame_id for p in values)))
    deltas=[(p[1]-p[3])/1e9 for p in values if p[3] is not None]
    report['pose_minus_latest_received_clock_seconds']={'min':min(deltas),'max':max(deltas)}
    report['pose_samples']=[]
    for i in [0,len(values)//2,len(values)-1]:
        _,stamp,m,_=values[i]; p=m.pose.position; q=m.pose.orientation
        report['pose_samples'].append({'sec':stamp//1000000000,'nanosec':stamp%1000000000,
                                      'xyz':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]})
print(json.dumps(report,indent=2))
node.destroy_node()
rclpy.shutdown()
