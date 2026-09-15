#!/usr/bin/env python3
"""One immutable-input async SLAM pilot. Existing output directory prevents repeats."""
import argparse,collections,csv,datetime,hashlib,json,math,os,re,signal,subprocess,time,traceback
from pathlib import Path
import yaml,rclpy,tf2_ros
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from visualization_msgs.msg import MarkerArray
from lifecycle_msgs.srv import ChangeState
from slam_toolbox.srv import SerializePoseGraph
from rosidl_runtime_py.convert import message_to_ordereddict
from evaluate import evaluate,read_estimate,load_reference,resource_summary,failure_fraction
from clock_guard import ClockGuard
WS=Path.home()/'slam_testing';HERE=Path(__file__).resolve().parent
BAG=WS/'bags/static/world_v0/r1/v0_r1_canonical';EXPECTED='f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715'
parser=argparse.ArgumentParser();parser.add_argument('--run-id',default='run_002');args=parser.parse_args()
assert re.fullmatch(r'run_[0-9]{3}',args.run_id),'Invalid run ID'
OUT=WS/'results/static/world_v0/r1'/args.run_id;CONFIG=HERE/'mapper_params_online_async.yaml'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def utc():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def ns(t):return t.sec*1000000000+t.nanosec
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def dump(p,v):p.write_text(json.dumps(v,indent=2))
def conflicting_processes():
 out=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit() or int(p.name)==os.getpid():continue
  try:
   exe=(p/'comm').read_text().strip();argv=(p/'cmdline').read_bytes().split(b'\0');args=b' '.join(argv).decode(errors='replace');base=Path(argv[0].decode(errors='replace')).name if argv[0] else ''
  except (OSError,ProcessLookupError):continue
  if base in ['async_slam_toolbox_node','sync_slam_toolbox_node','localization_slam_toolbox_node','lifelong_slam_toolbox_node','robot_state_publisher','parameter_bridge','gzserver','gzclient'] or (exe in ['ruby','gz','gzserver','gzclient'] and ('gz sim' in args or 'gzserver' in args or 'gzclient' in args)) or any(s in exe for s in ['slam_toolbox','robot_state_pub','parameter_bridg']) or ('ros2 launch turtlebot3' in args and exe not in ['bash','timeout']):out.append({'pid':int(p.name),'comm':exe,'args':args})
 return out
assert sha(BAG/'v0_r1_canonical_0.mcap')==EXPECTED,'Canonical checksum mismatch'
assert not OUT.exists(),'Output exists; no overwrite/repeat allowed'
manifest=json.loads((HERE/'configuration_manifest.json').read_text());assert sha(CONFIG)==manifest['configuration_sha256']
import xml.etree.ElementTree as ET
version=ET.parse('/opt/ros/jazzy/share/slam_toolbox/package.xml').getroot().findtext('version');assert version=='2.8.5'
conflicts=conflicting_processes();assert not conflicts,conflicts
OUT.mkdir(parents=True)
meta={'experiment_id':'V0_R1_'+args.run_id.upper(),'canonical_bag_sha256':EXPECTED,'slam_toolbox_version':version,'configuration_sha256':sha(CONFIG),'slam_testing_git_commit':subprocess.check_output(['git','-C',str(WS),'rev-parse','HEAD'],text=True).strip(),'git_status_at_start':subprocess.check_output(['git','-C',str(WS),'status','--porcelain'],text=True),'ros_distribution':os.environ.get('ROS_DISTRO'),'start_utc':utc(),'success':False,'trajectory_coverage':0.,'failure_reason':None,'failure_sim_time':None,'failure_route_fraction':None,'methodology':'V0_R1_v2','final_drain_wall_seconds':6.,'conflicting_processes_before':conflicts,'tooling_hashes':{p.name:sha(p) for p in HERE.iterdir() if p.is_file()},'configuration_changes':manifest['parameter_changes_from_installed']}
(OUT/'slam_configuration.yaml').write_bytes(CONFIG.read_bytes())
(OUT/'tooling_snapshot').mkdir()
for source in HERE.iterdir():
 if source.is_file():(OUT/'tooling_snapshot'/source.name).write_bytes(source.read_bytes())
reference=load_reference(BAG)
protocol=yaml.safe_load((BAG.parent/'metadata.yaml').read_text())
route_start_ns=protocol['actual_start_pose']['stamp_ns'];route_end_ns=protocol['actual_final_pose']['stamp_ns']
rclpy.init();node=rclpy.create_node('slam_pilot_monitor',parameter_overrides=[Parameter('use_sim_time',value=True)])
buffer=tf2_ros.Buffer(cache_time=Duration(seconds=30),node=node);listener=tf2_ros.TransformListener(buffer,node)
files=[]
def writer(name,header):
 f=(OUT/name).open('w');files.append(f);w=csv.writer(f);w.writerow(header);return w
est=writer('estimated_trajectory.csv',['timestamp','x','y','yaw'])
gt=writer('ground_truth_trajectory.csv',['timestamp','x','y','z','qx','qy','qz','qw'])
pp=writer('inserted_poses.csv',['timestamp','x','y','yaw'])
resources=writer('resources.csv',['utc','elapsed_wall_seconds','phase','simulation_ns','pid','cpu_seconds','cpu_percent_one_core','rss_bytes'])
graphcsv=writer('graph_series.csv',['simulation_ns','nodes','edges'])
clock=None;clocks=[];pending=[];gt_count=0;estimated_count=0;missing=[];inserted=[];scans=[];graphs=[];maps=[];last_grid=None
clock_guard=ClockGuard()
phase='startup';slam=player=None;last_resource=None;resource_data=[];last_poll=0.;last_tick=0.;startwall=time.monotonic();clock_authorities=set();authority_conflicts=[];subscriptions=[]
def onclock(m):
 global clock,phase
 if clock is None:phase='active_playback'
 t=ns(m.clock)
 if clock is not None and t<clock:raise RuntimeError('Simulation time reset')
 clock=t;clocks.append((time.monotonic(),t));clock_guard.clock_received(t,time.monotonic())
def ongt(m):
 global gt_count
 p,q=m.pose.position,m.pose.orientation;t=ns(m.header.stamp)
 gt.writerow([f'{t/1e9:.9f}',p.x,p.y,p.z,q.x,q.y,q.z,q.w]);gt_count+=1;pending.append(t)
def onpose(m):
 t=ns(m.header.stamp);p=m.pose.pose.position;q=m.pose.pose.orientation
 inserted.append(t);pp.writerow([f'{t/1e9:.9f}',p.x,p.y,yaw(q)])
def ongraph(m):
 nodes=[x for x in m.markers if x.ns=='slam_toolbox' and x.action==0]
 edges=[x for x in m.markers if x.ns=='slam_toolbox_edges' and x.action==0]
 g={'simulation_ns':clock,'nodes':len(nodes),'edges':sum(len(x.points)//2 for x in edges),'node_ids':[x.id for x in nodes]}
 graphs.append(g);graphcsv.writerow([clock,g['nodes'],g['edges']]);dump(OUT/'graph_markers_latest.json',message_to_ordereddict(m))
def onmap(m):
 global last_grid
 last_grid=m;maps.append({'stamp_ns':ns(m.header.stamp),'width':m.info.width,'height':m.info.height})
qos=QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT)
for typ,topic,cb,q in [(Clock,'/clock',onclock,qos),(PoseStamped,'/ground_truth/pose',ongt,qos),(LaserScan,'/scan',lambda m:scans.append(ns(m.header.stamp)),qos),(PoseWithCovarianceStamped,'/pose',onpose,1000),(MarkerArray,'/slam_toolbox/graph_visualization',ongraph,10),(OccupancyGrid,'/map',onmap,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:subscriptions.append(node.create_subscription(typ,topic,cb,q))
def sample_resource():
 global last_resource
 if not slam or slam.poll() is not None:return
 try:
  stat=Path(f'/proc/{slam.pid}/stat').read_text().rsplit(')',1)[1].split();cpu=(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK');rss=int(stat[21])*os.sysconf('SC_PAGE_SIZE')
 except OSError:return
 now=time.monotonic();pct=0. if last_resource is None else 100*(cpu-last_resource[1])/(now-last_resource[0]);last_resource=(now,cpu)
 r=[utc(),now-startwall,phase,clock,slam.pid,cpu,pct,rss];resources.writerow(r);resource_data.append(r)
def check_clock_publishers(now):
 pubs=node.get_publishers_info_by_topic('/clock')
 endpoints=[{'node':p.node_name,'gid':bytes(p.endpoint_gid).hex()} for p in pubs]
 clock_authorities.update((p['node'],p['gid']) for p in endpoints)
 event=clock_guard.observe(endpoints,now-startwall)
 if event['warning']:
  node.get_logger().warning(event['warning']);print(event['warning'],flush=True)
 if event['conflict']:
  authority_conflicts.append(event);raise RuntimeError(event['conflict'])
 return event

def tick():
 global pending,estimated_count,last_poll,last_tick
 now=time.monotonic()
 if now-last_tick>.5:
  sample_resource();keep=[]
  for t in pending:
   if buffer.can_transform('map','base_footprint',Time(nanoseconds=t)):
    m=buffer.lookup_transform('map','base_footprint',Time(nanoseconds=t));p=m.transform.translation;q=m.transform.rotation
    est.writerow([f'{t/1e9:.9f}',p.x,p.y,yaw(q)]);estimated_count+=1
   elif clock and clock-t>2000000000:missing.append(t)
   else:keep.append(t)
  pending=keep;last_tick=now
 if now-last_poll>.5:
  check_clock_publishers(now)
  if player is not None and player.poll() is None:clock_guard.check_advancing(now)
  last_poll=now
 if slam and phase not in ['shutdown','done'] and slam.poll() is not None:raise RuntimeError('SLAM exited unexpectedly')
def spin_until(pred,timeout=30):
 end=time.monotonic()+timeout
 while not pred():
  if time.monotonic()>end:raise RuntimeError('Runtime wait timed out')
  rclpy.spin_once(node,timeout_sec=.01);tick()
def spin_wall(seconds):
 end=time.monotonic()+seconds;spin_until(lambda:time.monotonic()>=end,seconds+2)
def call(typ,name,req,timeout=30):
 client=node.create_client(typ,name);spin_until(lambda:client.service_is_ready(),timeout)
 fut=client.call_async(req);spin_until(fut.done,timeout);ans=fut.result();node.destroy_client(client);return ans
def change(i):
 req=ChangeState.Request();req.transition.id=i;ans=call(ChangeState,'/slam_toolbox/change_state',req);assert ans.success,('lifecycle transition',i)
 meta.setdefault('lifecycle_transitions',[]).append(i)
def stop(proc):
 if proc and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10);raise RuntimeError('Forced process shutdown needed')
sl=pl=None
try:
 spin_wall(2)
 ep={t:[p.node_name for p in node.get_publishers_info_by_topic(t)] for t in ['/clock','/scan','/odom','/tf','/tf_static','/ground_truth/pose','/map','/pose']}
 meta['publishers_before']=ep;check_clock_publishers(time.monotonic());assert not any(v for t,v in ep.items() if t!='/clock'),ep
 command=['/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node','--ros-args','--params-file',str(CONFIG),'-r','__node:=slam_toolbox']
 meta['slam_launch_command']=command;sl=(OUT/'slam_toolbox.log').open('w')
 slam=subprocess.Popen(command,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True);meta['slam_pid']=slam.pid;meta['slam_start_utc']=utc();slam_start=time.monotonic();sample_resource()
 change(1);change(3);phase='startup';spin_wall(1)
 meta['slam_subscriptions']=node.get_subscriber_names_and_types_by_node('slam_toolbox','/')
 assert not any(t=='/ground_truth/pose' for t,ty in meta['slam_subscriptions'])
 with (OUT/'effective_parameters.yaml').open('w') as f:
  subprocess.run(['ros2','param','dump','/slam_toolbox'],stdout=f,check=True,timeout=20)
 command=['ros2','bag','play',str(BAG),'--rate','1.0','--clock','100','--delay','3','--topics','/scan','/odom','/tf','/tf_static','/ground_truth/pose']
 meta['playback_command']=command;pl=(OUT/'playback.log').open('w');phase='startup';play_start=time.monotonic();meta['playback_start_utc']=utc()
 meta['clock_immediately_before_player']=check_clock_publishers(time.monotonic())
 clock_guard.start_playback(time.monotonic())
 player=subprocess.Popen(command,stdout=pl,stderr=subprocess.STDOUT,start_new_session=True)
 print('PILOT RUNNING: SLAM PID '+str(slam.pid)+'; immutable bag playing once.',flush=True)
 spin_until(lambda:player.poll() is not None,220);meta['playback_process_wall_seconds']=time.monotonic()-play_start;meta['playback_returncode']=player.returncode;assert player.returncode==0
 meta['playback_end_utc']=utc();phase='finalization';spin_wall(6)
 print('Playback finished; final drain complete; saving map and graph.',flush=True)
 phase='finalization'
 ans=call(GetMap,'/slam_toolbox/dynamic_map',GetMap.Request());m=ans.map;assert m.info.width>0 and m.info.height>0,'No map'
 # Standard trinary image, explicitly documented thresholds; keep raw grid too.
 w,h=m.info.width,m.info.height;pixels=bytearray()
 for y in range(h-1,-1,-1):
  for x in range(w):
   value=m.data[y*w+x];pixels.append(205 if value<0 else (0 if value/100>.65 else (254 if value/100<.196 else 205)))
 (OUT/'map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+pixels)
 origin=m.info.origin
 (OUT/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','mode':'trinary','resolution':m.info.resolution,'origin':[origin.position.x,origin.position.y,yaw(origin.orientation)],'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))
 dump(OUT/'occupancy_grid.json',message_to_ordereddict(m));meta['saved_map']={'width':w,'height':h,'stamp_ns':ns(m.header.stamp),'resolution':m.info.resolution,'method':'GetMap /slam_toolbox/dynamic_map; vertically flipped P5 trinary export; raw grid retained'}
 req=SerializePoseGraph.Request();req.filename=str(OUT/'slam_graph');ans=call(SerializePoseGraph,'/slam_toolbox/serialize_map',req,40);meta['serialize_result']=ans.result;assert ans.result==0
 for ext in ['posegraph','data']:assert (OUT/('slam_graph.'+ext)).stat().st_size>0
 spin_wall(.5);sample_resource();phase='shutdown';change(4);change(6);sample_resource();stop(slam)
 meta['slam_process_wall_seconds']=time.monotonic()-slam_start;meta['slam_returncode']=slam.returncode;assert slam.returncode==0
 meta['success']=True
except Exception as ex:
 meta['failure_reason']=str(ex);meta['failure_sim_time']=clock/1e9 if clock is not None else None
 meta['failure']=str(ex);meta['traceback']=traceback.format_exc();print('PILOT FAILED (no repeat): '+str(ex),flush=True)
finally:
 phase='shutdown'
 for proc in [player,slam]:
  try:stop(proc)
  except Exception as ex:meta.setdefault('shutdown_errors',[]).append(str(ex));meta['success']=False
 for f in files:f.close()
 if sl:sl.close()
 if pl:pl.close()
 meta['end_utc']=utc();meta['total_wall_seconds']=time.monotonic()-startwall
 meta['bag_sha256_after']=sha(BAG/'v0_r1_canonical_0.mcap');meta['configuration_sha256_after']=sha(CONFIG)
 meta['clock_authorities']=[{'node':n,'gid':g} for n,g in clock_authorities];meta['competing_clock_events']=authority_conflicts
 meta['clock_guard']={'snapshots':clock_guard.snapshots,'advance_count':clock_guard.advances,'passed':clock_guard.passed()}
 meta['scan_messages_observed']=len(scans);meta['gt_messages']=gt_count;meta['estimated_samples']=estimated_count;meta['missing_estimated_timestamps_ns']=missing+pending
 meta['inserted_pose_publications']=len(inserted);meta['inserted_scan_stamps_ns']=inserted;meta['maps_published']=maps;meta['graph_snapshots']=graphs
 if len(clocks)>1:
  meta['clock']={'messages':len(clocks),'first_ns':clocks[0][1],'last_ns':clocks[-1][1],'wall_span_seconds':clocks[-1][0]-clocks[0][0],'simulation_span_seconds':(clocks[-1][1]-clocks[0][1])/1e9,'real_time_factor':((clocks[-1][1]-clocks[0][1])/1e9)/(clocks[-1][0]-clocks[0][0])}
 meta['bag_simulation_duration_seconds']=170.884
 meta['resource_metrics']=resource_summary(OUT/'resources.csv',clocks[0][0]-startwall if clocks else None,clocks[-1][0]-startwall if clocks else None)
 meta['resource_clock_boundaries_elapsed_wall_seconds']=[clocks[0][0]-startwall,clocks[-1][0]-startwall] if clocks else None
 try:
  assessment=evaluate(read_estimate(OUT/'estimated_trajectory.csv'),reference,OUT/'evaluation')
  meta['trajectory_coverage']=assessment['trajectory_coverage']
  dump(OUT/'evaluation'/'trajectory_metrics.json',assessment)
  meta['trajectory_evaluation']=assessment
 except Exception as ex:
  meta['success']=False;meta['failure_reason']=meta['failure_reason'] or 'Evaluation failed: '+str(ex)
  meta['trajectory_evaluation']={'available':False,'error':str(ex)}
 dump(OUT/'resource_metrics.json',meta['resource_metrics'])
 logs=(OUT/'slam_toolbox.log').read_text() if (OUT/'slam_toolbox.log').exists() else ''
 drops=[line for line in logs.splitlines() if 'Message Filter dropping' in line]
 stats={'input_scans_in_bag':855,'scan_topic_messages_seen_by_monitor':len(scans),'slam_callback_entry_count':None,'exact_TF_rejection_count':None,'exact_time_motion_skip_count':None,'successful_scan_insertions_from_pose_publications':len(inserted),'first_successful_stamp_ns':inserted[0] if inserted else None,'filter_drop_log_lines':drops,'queue_full_log_count':sum('queue is full' in x for x in drops),'out_the_back_log_count':sum('earlier than all' in x for x in drops),'odom_pose_failure_log_count':logs.count('Failed to compute odom pose'),'laser_device_rejection_log_count':logs.count('Failed to create laser device'),'warn_error_fatal_lines':[l for l in logs.splitlines() if any(s in l for s in ['[WARN]','[ERROR]','[FATAL]'])],'filter_log_limitation':'tf2 MessageFilter logs are throttled at 2500 ms; log counts are not exact drop counters','silent_skip_counts':'Unavailable: shouldProcessScan time/motion/initial-stabilization gates and Karto rejections lack public counters. throttle_scans=1 disables modulo skipping.','loop_closure_events':'Unavailable through current default public logs/interfaces; no inference of zero from absent messages.'}
 dump(OUT/'scan_statistics.json',stats)
 meta['artifacts']={p.name:{'bytes':p.stat().st_size,'sha256':sha(p)} for p in OUT.iterdir() if p.is_file()}
 meta['validation']={'received_all_855_scans_at_monitor':len(scans)==855,'graph_insertions_observed':len(inserted)>0,'map_produced':bool(maps),'grid_saved':(OUT/'map.pgm').exists() and (OUT/'map.yaml').exists(),'graph_serialized':meta.get('serialize_result')==0,'estimated_trajectory_recorded':estimated_count>0,'monitoring_recorded':len(resource_data)>1,'resource_phase_separation':bool(meta['resource_metrics'].get('active_playback')),'trajectory_evaluation_succeeded':meta['trajectory_evaluation'].get('available',False) and meta['trajectory_evaluation'].get('rpe_translation_1m_m',{}).get('samples',0)>0,'laser_range_warning_absent':'maximum laser range setting (20.0 m)' not in logs,'sole_rosbag_clock':clock_guard.passed(),'ground_truth_evaluation_only':not any(t=='/ground_truth/pose' for t,ty in meta.get('slam_subscriptions',[])),'canonical_unchanged':meta['bag_sha256_after']==EXPECTED,'configuration_unchanged':meta['configuration_sha256_after']==manifest['configuration_sha256'],'no_unhandled_errors':not meta.get('failure') and not meta.get('shutdown_errors')}
 meta['success']=meta['success'] and all(meta['validation'].values())
 if not meta['success']:
  meta['failure_reason']=meta['failure_reason'] or '; '.join(k for k,v in meta['validation'].items() if not v) or str(meta.get('shutdown_errors','Run failed'))
  if meta['failure_sim_time'] is None and clock is not None:meta['failure_sim_time']=clock/1e9
  meta['failure_route_fraction']=failure_fraction(reference,round(meta['failure_sim_time']*1e9) if meta['failure_sim_time'] is not None else None,route_start_ns,route_end_ns)
 else:
  meta['failure_reason']=meta['failure_sim_time']=meta['failure_route_fraction']=None
 dump(OUT/'run_metadata.json',meta)
 listener.unregister();node.destroy_node();rclpy.shutdown()
 print(json.dumps({'success':meta['success'],'resources':meta.get('resource_metrics'),'evaluation':meta.get('trajectory_evaluation'),'scans':stats,'graph_latest':graphs[-1] if graphs else None,'estimated_samples':estimated_count},indent=2),flush=True)
if not meta['success']:raise SystemExit(1)
