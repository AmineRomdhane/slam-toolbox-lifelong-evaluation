"""Frozen v2: exact simulation-time association, rigid SE(2), 1 m GT-arc RPE."""
import bisect,csv,hashlib,json,math,statistics
from decimal import Decimal
from pathlib import Path

def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def ns(s):return int(Decimal(str(s))*1000000000)
def metrics(a):
 if not a:return {'rmse':None,'median':None,'p95':None,'samples':0}
 s=sorted(a);i=(len(s)-1)*.95;j=int(i)
 return {'rmse':math.sqrt(sum(x*x for x in a)/len(a)),'median':statistics.median(a),'p95':s[j]+(s[min(j+1,len(s)-1)]-s[j])*(i-j),'samples':len(a)}
def read_estimate(path):
 out=[]
 for r in csv.DictReader(Path(path).read_text().splitlines()):out.append((ns(r['timestamp']),float(r['x']),float(r['y']),float(r['yaw'])))
 if any(b[0]<=a[0] for a,b in zip(out,out[1:])):raise ValueError('Estimated timestamps must be strictly increasing')
 if not all(all(math.isfinite(x) for x in row) for row in out):raise ValueError('Nonfinite trajectory')
 return out

def load_reference(bag):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from geometry_msgs.msg import PoseStamped
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 reader.set_filter(rosbag2_py.StorageFilter(topics=['/ground_truth/pose']))
 out=[]
 while reader.has_next():
  _,data,_=reader.read_next();m=deserialize_message(data,PoseStamped);p,q=m.pose.position,m.pose.orientation
  assert m.header.frame_id=='gazebo_world'
  out.append((m.header.stamp.sec*1000000000+m.header.stamp.nanosec,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
 assert out and all(b[0]>a[0] for a,b in zip(out,out[1:]))
 return out

def fit_se2(est,gt):
 n=len(est)
 if n<2:raise ValueError('Fewer than two associated poses')
 ex=sum(p[1] for p in est)/n;ey=sum(p[2] for p in est)/n;gx=sum(p[1] for p in gt)/n;gy=sum(p[2] for p in gt)/n
 dot=sum((p[1]-ex)*(q[1]-gx)+(p[2]-ey)*(q[2]-gy) for p,q in zip(est,gt))
 cross=sum((p[1]-ex)*(q[2]-gy)-(p[2]-ey)*(q[1]-gx) for p,q in zip(est,gt))
 if math.hypot(dot,cross)<1e-12:raise ValueError('Degenerate positional alignment')
 theta=math.atan2(cross,dot);c,s=math.cos(theta),math.sin(theta)
 return theta,gx-c*ex+s*ey,gy-s*ex-c*ey

def arc(gt):
 out=[0.]
 for a,b in zip(gt,gt[1:]):out.append(out[-1]+math.hypot(b[1]-a[1],b[2]-a[2]))
 return out

def evaluate(est,gt,directory):
 directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
 lookup={p[0]:p for p in est};reference_times={g[0] for g in gt};matched=[i for i,g in enumerate(gt) if g[0] in lookup]
 report={'methodology':'V0_R1_v2','reference_samples':len(gt),'estimated_samples':len(est),'associated_samples':len(matched),'trajectory_coverage':len(matched)/len(gt) if gt else 0.,'unmatched_estimated_samples':sum(p[0] not in reference_times for p in est),'available':False}
 if not matched:return report
 report['coverage_time_fraction']=sum((b[0]-a[0]) for a,b in zip(gt,gt[1:]) if a[0] in lookup and b[0] in lookup)/(gt[-1][0]-gt[0][0])
 e=[lookup[gt[i][0]] for i in matched];g=[gt[i] for i in matched]
 try:theta,tx,ty=fit_se2(e,g)
 except ValueError as err:report['unavailable_reason']=str(err);return report
 report['alignment']={'theta_rad':theta,'tx_m':tx,'ty_m':ty,'scale':1.0,'direction':'estimated map coordinates -> gazebo_world','fit':'Least-squares XY over all exactly associated samples, equal sample weight; yaw excluded from fitting; no reflection, robust trimming or scale correction'}
 c,s=math.cos(theta),math.sin(theta);translation=[];angles=[]
 with (directory/'absolute_errors.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['timestamp_ns','aligned_x','aligned_y','gt_x','gt_y','dx_m','dy_m','translation_error_m','yaw_error_rad','abs_yaw_error_rad'])
  for p,q in zip(e,g):
   x=c*p[1]-s*p[2]+tx;y=s*p[1]+c*p[2]+ty;dx=x-q[1];dy=y-q[2];d=math.hypot(dx,dy);a=wrap(p[3]+theta-q[3]);translation.append(d);angles.append(abs(a));w.writerow([p[0],x,y,q[1],q[2],dx,dy,d,a,abs(a)])
 distances=arc(gt);invalid=[0]
 for k in range(len(gt)-1):invalid.append(invalid[-1]+int(gt[k][0] not in lookup or gt[k+1][0] not in lookup or gt[k+1][0]-gt[k][0]>100000000))
 rpe_t=[];rpe_r=[]
 def interp(a,b,f):return (a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]),wrap(a[3]+f*wrap(b[3]-a[3])))
 def relative(a,b):
  dx=b[0]-a[1];dy=b[1]-a[2];c=math.cos(a[3]);s=math.sin(a[3]);return c*dx+s*dy,-s*dx+c*dy,wrap(b[2]-a[3])
 with (directory/'rpe_1m_errors.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['start_timestamp_ns','end_timestamp_ns','gt_distance_m','translation_error_m','rotation_error_rad','abs_rotation_error_rad'])
  for i in matched:
   target=distances[i]+1.
   if target>distances[-1]:continue
   j=bisect.bisect_left(distances,target)
   if j<=i or invalid[j]!=invalid[i]:continue
   fraction=(target-distances[j-1])/(distances[j]-distances[j-1]);endtime=round(gt[j-1][0]+fraction*(gt[j][0]-gt[j-1][0]))
   gend=interp(gt[j-1],gt[j],fraction);eend=interp(lookup[gt[j-1][0]],lookup[gt[j][0]],fraction)
   gr=relative(gt[i],gend);er=relative(lookup[gt[i][0]],eend)
   d=math.hypot(er[0]-gr[0],er[1]-gr[1]);a=wrap(er[2]-gr[2]);rpe_t.append(d);rpe_r.append(abs(a));w.writerow([gt[i][0],endtime,1.,d,a,abs(a)])
 report.update(available=True,ate_translation_m=metrics(translation),ape_yaw_rad=metrics(angles),rpe_translation_1m_m=metrics(rpe_t),rpe_rotation_1m_rad=metrics(rpe_r),rpe_delta_m=1.)
 return report

def resource_summary(raw,first_clock_wall,last_clock_wall):
 rr=list(csv.DictReader(Path(raw).read_text().splitlines()));r=[{k:float(row[k]) for k in ['elapsed_wall_seconds','cpu_seconds','rss_bytes']} for row in rr]
 if len(r)<2:return {'available':False}
 start=r[0]['elapsed_wall_seconds'];end=r[-1]['elapsed_wall_seconds']
 def window(lo,hi):
  duration=cpu=rss=0.;peaks=[];mem=[]
  if lo is None or hi is None:return None
  for a,b in zip(r,r[1:]):
   ta=a['elapsed_wall_seconds'];tb=b['elapsed_wall_seconds'];overlap=max(0.,min(tb,hi)-max(ta,lo))
   if overlap<=0:continue
   rate=(b['cpu_seconds']-a['cpu_seconds'])/(tb-ta);duration+=overlap;cpu+=rate*overlap;rss+=a['rss_bytes']*overlap;peaks.append(rate*100);mem.extend([a['rss_bytes'],b['rss_bytes']])
  return {'start_elapsed_wall_seconds':lo,'end_elapsed_wall_seconds':hi,'observed_seconds':duration,'mean_cpu_percent_one_core':100*cpu/duration if duration else None,'peak_cpu_percent_one_core':max(peaks) if peaks else None,'mean_rss_bytes':rss/duration if duration else None,'peak_rss_bytes':max(mem) if mem else None}
 active=first_clock_wall is not None and last_clock_wall is not None
 return {'available':True,'primary_window':'active_playback','startup':window(start,first_clock_wall if active else end),'active_playback':window(first_clock_wall,last_clock_wall),'finalization':window(last_clock_wall,end) if active else None,'whole_process_sampled':window(start,end),'method':'Overlap-weighted integration of proc cumulative CPU deltas; RSS left-held; peaks from intervals overlapping window (RSS includes interval endpoints). 100% CPU = one core. Bounds use first/last received replay clock; whole_process_sampled excludes unsampled process creation/exit tails.'}

def failure_fraction(gt,simulation_ns,start_ns,end_ns):
 if simulation_ns is None:return 0.
 times=[x[0] for x in gt];d=arc(gt)
 def at(t):
  j=bisect.bisect_right(times,t)-1
  if j<0:return d[0]
  if j>=len(times)-1:return d[-1]
  return d[j]+(d[j+1]-d[j])*(t-times[j])/(times[j+1]-times[j])
 den=at(end_ns)-at(start_ns)
 return min(1.,max(0.,(at(min(end_ns,max(start_ns,simulation_ns)))-at(start_ns))/den)) if den else None
