"""Prepare exact static assets and canonical GT sensor poses; never read odometry/SLAM."""
import argparse,bisect,csv,hashlib,json,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
WS=Path(__file__).resolve().parents[3]; TB=Path.home()/'turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo'
EXPECTED='f094c269194313d72ac49d9f7ab181bca5c301763453d1e9d38505072b26a715'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def prepare(out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);bag=WS/'bags/static/world_v0/r1/v0_r1_canonical';assert sha(bag/'v0_r1_canonical_0.mcap')==EXPECTED
 inspection=json.loads((WS/'benchmark/references/world_v0/input_inspection.json').read_text())
 for p,h in inspection['asset_sha256'].items():assert sha(p)==h,p
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan','/ground_truth/pose']))
 scans=[];gt=[]
 while reader.has_next():
  t,b,_=reader.read_next();m=deserialize_message(b,LaserScan if t=='/scan' else PoseStamped);ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  if t=='/scan':
   assert len(m.ranges)==360 and m.header.frame_id=='base_scan' and m.range_max==3.5
   assert np.float32(m.angle_max)==np.float32(6.28) and np.float32(m.angle_increment)==np.float32(6.28/359)
   assert np.float32(m.range_min)==np.float32(.12) and m.angle_min==0
   scans.append(ns)
  else:
   assert m.header.frame_id=='gazebo_world';p=m.pose.position;q=m.pose.orientation;gt.append([ns,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
 times=[r[0] for r in gt];assert len(scans)==855 and len(gt)==10041 and all(b>a for a,b in zip(times,times[1:]));poses=[];associations=[];robot=[]
 for t in scans:
  j=bisect.bisect_left(times,t)
  if j==len(times) or j==0 and t!=times[0]:associations.append({'stamp_ns':t,'method':'omitted_no_bracket'});continue
  if times[j]==t:p=np.array(gt[j][1:4]);q=np.array(gt[j][4:]);kind='exact';i=j;alpha=0.
  else:
   i=j-1;alpha=(t-times[i])/(times[j]-times[i]);p=(1-alpha)*np.array(gt[i][1:4])+alpha*np.array(gt[j][1:4]);q=Slerp([0,1],Rotation.from_quat([gt[i][4:],gt[j][4:]]))([alpha]).as_quat()[0];kind='interpolated'
  rot=Rotation.from_quat(q);q=rot.as_quat();lidar=p+rot.apply([-.032,0,.171]);poses.append([t,*lidar,*q]);robot.append([t,*p,*q]);associations.append({'stamp_ns':t,'method':kind,'left_ns':times[i],'right_ns':times[j],'alpha':alpha})
 counts={k:sum(a['method']==k for a in associations) for k in ['exact','interpolated','omitted_no_bracket']};assert counts=={'exact':49,'interpolated':805,'omitted_no_bracket':1};assert associations[0]['stamp_ns']==11000000000 and associations[0]['method']=='omitted_no_bracket'
 for name,rows in [('lidar_poses.csv',poses),('robot_poses.csv',robot),('canonical_gt.csv',gt)]:
  with (out/name).open('w') as f:w=csv.writer(f);w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw']);w.writerows(rows)
 (out/'interpolation.json').write_text(json.dumps({'counts':counts,'extrapolation':False,'method':'Integer-ns brackets; linear XYZ; normalized shortest-arc quaternion SLERP; full SE(3) composition','max_gt_gap_ns':max(b-a for a,b in zip(times,times[1:])),'associations':associations},indent=2))
 world=ET.parse(TB/'worlds/turtlebot3_world.world');w=world.find('world');fuel=Path.home()/'.gz/fuel/fuel.gazebosim.org/openrobotics/models'
 for inc in w.findall('include'):
  uri=inc.find('uri');uri.text=str(fuel/('ground plane/5/model.sdf' if uri.text.endswith('Ground Plane') else 'sun/3/model.sdf'))
 sensor=ET.parse(TB/'models/turtlebot3_burger/model.sdf').find('.//sensor[@type="gpu_lidar"]');sensor.set('name','ideal_lidar');sensor.find('pose').text='0 0 0 0 0 0';sensor.find('topic').text='/reference/scan';sensor.find('visualize').text='false';sensor.find('gz_frame_id').text='physical_lidar';sensor.find('lidar/noise/mean').text='0';sensor.find('lidar/noise/stddev').text='0'
 # Sampling frequency affects offline waiting only; angular geometry/range remain untouched.
 sensor.find('update_rate').text='100'
 template=ET.Element('sdf',version='1.8');model=ET.SubElement(template,'model',name='reference_@INDEX@');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='sensor_carrier');sensor.find('pose').text='@POSE@';sensor.find('gz_frame_id').text='reference_@INDEX@';link.append(sensor)
 ET.ElementTree(template).write(out/'reference_sensor.sdf',encoding='unicode',xml_declaration=True)
 world.write(out/'reference_world.sdf',encoding='unicode',xml_declaration=True)
 provenance={'canonical_bag_sha256':EXPECTED,'asset_sha256':inspection['asset_sha256'],'turtlebot3_simulations_commit':inspection['turtlebot3_simulations_commit'],'fuel_provenance_limit':inspection['asset_provenance_limit'],'physical_extrinsic_xyz':[-.032,0,.171],'ros_extrinsic_xyz':[-.032,0,.182],'z_discrepancy_m':.011,'normal_noise_stddev_m':.01,'reference_noise_stddev_m':0,'offline_sensor_update_rate_hz':100,'sdf_angle_increment_rad':6.28/359,'ros_float32_angle_increment_rad':float(np.float32(6.28/359)),'geometry_note':'Physical SDF angles (double); canonical ROS fields are float32 representations. GPU sensor uses exact original SDF geometry.','reference_world_sha256':sha(out/'reference_world.sdf'),'scan_count':854,'omitted':1,'rays':854*360,'inspection':inspection['sdf_lidar']}
 (out/'provenance.json').write_text(json.dumps(provenance,indent=2));print(counts)
if __name__=='__main__':p=argparse.ArgumentParser();p.add_argument('output');prepare(p.parse_args().output)
