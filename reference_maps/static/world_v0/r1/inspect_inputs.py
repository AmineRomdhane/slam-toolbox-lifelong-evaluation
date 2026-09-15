"""Read canonical inputs only; write an inspection report, not an occupancy map."""
import bisect,hashlib,json,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
W=Path('/home/roam5170/slam_testing'); T=Path('/home/roam5170/turtlebot3_ws/src/turtlebot3_simulations'); G=T/'turtlebot3_gazebo'; B=W/'bags/static/world_v0/r1/v0_r1_canonical'; O=W/'reference_maps/static/world_v0/r1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
expected='f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715';assert sha(B/'v0_r1_canonical_0.mcap')==expected
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/scan','/ground_truth/pose','/tf_static']));types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};scans=[];gt=[];tf={};sensor=None
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
while r.has_next():
 topic,data,_=r.read_next();m=deserialize_message(data,types[topic])
 if topic=='/scan':
  params={k:getattr(m,k) for k in ['angle_min','angle_max','angle_increment','range_min','range_max','scan_time','time_increment']};params.update(frame=m.header.frame_id,samples=len(m.ranges))
  if sensor is None:sensor=params
  assert params==sensor;scans.append(stamp(m))
 elif topic=='/ground_truth/pose':
  assert m.header.frame_id=='gazebo_world';p=m.pose.position;q=m.pose.orientation;gt.append([stamp(m),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
 else:
  for t in m.transforms:
   key=t.header.frame_id.lstrip('/')+' -> '+t.child_frame_id.lstrip('/');v=t.transform.translation;q=t.transform.rotation;value={'translation_m':[v.x,v.y,v.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]}
   if key in tf:assert tf[key]==value
   tf[key]=value
times=[g[0] for g in gt];assert all(b>a for a,b in zip(times,times[1:]));assert all(b>a for a,b in zip(scans,scans[1:]));exact=sum(t in set(times) for t in scans);outside=[t for t in scans if t<times[0] or t>times[-1]]
root=ET.parse(G/'models/turtlebot3_burger/model.sdf');lidar=root.find('.//sensor[@type="gpu_lidar"]/lidar');visuals=[]
for v in ET.parse(G/'models/turtlebot3_world/model.sdf').findall('.//visual'):
 visuals.append({'name':v.attrib['name'],'pose':v.findtext('pose','0 0 0 0 0 0'),'geometry_xml':ET.tostring(v.find('geometry'),encoding='unicode').strip()})
assets=[G/'worlds/turtlebot3_world.world',G/'models/turtlebot3_burger/model.sdf',G/'urdf/turtlebot3_burger.urdf',G/'launch/spawn_turtlebot3.launch.py',G/'launch/robot_state_publisher.launch.py',G/'launch/turtlebot3_world.launch.py']+list((G/'models/turtlebot3_world').rglob('*'))
fuel=Path('/home/roam5170/.gz/fuel/fuel.gazebosim.org/openrobotics/models')
assets+=list((fuel/'ground plane/5').glob('*.sdf'))+list((fuel/'sun/3').glob('*.sdf'))+list((fuel/'ground plane/5').glob('*.config'))+list((fuel/'sun/3').glob('*.config'))
report={'status':'BLOCKED_PENDING_FIRST_SCAN_POSE_POLICY','reference_generated':False,'evaluator_implemented':False,'canonical_bag_sha256':expected,'inspection_git_commit':subprocess.check_output(['git','-C',str(W),'rev-parse','HEAD'],text=True).strip(),'turtlebot3_simulations_commit':subprocess.check_output(['git','-C',str(T),'rev-parse','HEAD'],text=True).strip(),'turtlebot3_simulations_worktree':subprocess.check_output(['git','-C',str(T),'status','--porcelain'],text=True).strip(),'canonical_scan':sensor,'scan_count':len(scans),'expected_ray_count_all_scans':len(scans)*sensor['samples'],'scan_timestamp_ns':scans,'gt_count':len(gt),'first_gt':gt[0],'last_gt':gt[-1],'exact_gt_scan_associations':exact,'interpolatable_scan_count':len(scans)-exact-len(outside),'outside_gt_support_scan_ns':outside,'max_gt_gap_ns':max(b-a for a,b in zip(times,times[1:])),'static_tf':tf,'sdf_lidar':{'horizontal_samples':int(lidar.findtext('scan/horizontal/samples')),'horizontal_resolution':float(lidar.findtext('scan/horizontal/resolution')),'angle_min':float(lidar.findtext('scan/horizontal/min_angle')),'angle_max':float(lidar.findtext('scan/horizontal/max_angle')),'range_min':float(lidar.findtext('range/min')),'range_max':float(lidar.findtext('range/max')),'range_resolution':float(lidar.findtext('range/resolution')),'noise_type':lidar.findtext('noise/type'),'noise_mean':float(lidar.findtext('noise/mean')),'noise_stddev':float(lidar.findtext('noise/stddev')),'physical_sensor_in_base_footprint_xyz_m':[-.032,0,.171],'physical_sensor_in_base_footprint_quaternion_xyzw':[0,0,0,1],'extrinsic_evidence':'sdformat14 SemanticPose.Resolve(base_footprint) on installed model; inspection probe retained','ros_base_scan_in_base_footprint_xyz_m':[-.032,0,.182],'ros_minus_physical_z_m':.011},'world_visuals':visuals,'asset_sha256':{str(p):sha(p) for p in assets if p.is_file()},'gazebo_service_probe':json.loads(Path('/tmp/v0_reference_probe/services.json').read_text()),'proposed_interpolation':'Integer-nanosecond bracketing; linear XYZ interpolation and normalized shortest-arc quaternion SLERP; compose physical sensor extrinsic in full SE(3). No extrapolation unless explicitly approved.','boundary_policy':None,'ray_casting_status':'Headless static-world loading and pose/control service availability verified; GPU lidar output and full deterministic generation not yet tested.','asset_provenance_limit':'Robot/world source commit matches canonical metadata. Fuel assets are currently cached Ground Plane v5 and Sun v3; canonical metadata does not record their content hashes. Current hashes retained, no assets downloaded.','protected_inputs_modified':False}
O.mkdir(parents=True,exist_ok=True)
for name in ['input_inspection.json','inspect_inputs.py']:
 assert not (O/name).exists(),f'Refusing overwrite {name}'
(O/'input_inspection.json').write_text(json.dumps(report,indent=2));(O/'inspect_inputs.py').write_bytes(Path(__file__).read_bytes())
for name in ['inspect.cpp','CMakeLists.txt','probe.py','services.json','server.log']:(O/name).write_bytes((Path('/tmp/v0_reference_probe')/name).read_bytes())
print(json.dumps({k:report[k] for k in ['status','scan_count','gt_count','exact_gt_scan_associations','interpolatable_scan_count','outside_gt_support_scan_ns','max_gt_gap_ns']},indent=2))
