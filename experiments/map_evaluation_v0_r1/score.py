"""Offline fixed-alignment map scoring. No alignment fitting or ROS runtime."""
import argparse,csv,hashlib,json,math,statistics,subprocess
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
WS=Path('/home/roam5170/slam_testing');REF=WS/'reference_maps/static/world_v0/r1';ROOT=WS/'results/static/world_v0/r1'
HASH='e6bb2022935e23aa08002deddd5a319249e01d6d13b3ae4fcaeb18380d215272'
METRICS=['occupied_precision','occupied_recall','occupied_f1','occupied_iou','boundary_distance_mean_m','boundary_distance_median_m','boundary_distance_p95_m']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load_map(path):
 path=Path(path);m=yaml.safe_load(path.read_text());data=(path.parent/m['image']).read_bytes();idx=0;tokens=[]
 while len(tokens)<4:
  while data[idx:idx+1].isspace():idx+=1
  if data[idx:idx+1]==b'#':idx=data.index(b'\n',idx)+1;continue
  j=idx
  while not data[idx:idx+1].isspace():idx+=1
  tokens.append(data[j:idx])
 assert tokens[0]==b'P5' and tokens[3]==b'255';w,h=map(int,tokens[1:3]);idx+=2 if data[idx:idx+2]==b'\r\n' else 1
 assert len(data)-idx==w*h
 pixels=np.flipud(np.frombuffer(data[idx:],np.uint8).reshape(h,w));prob=pixels.astype(float)/255 if m['negate'] else (255-pixels.astype(float))/255
 assert m.get('mode','trinary')=='trinary' and m['resolution']>0
 a=np.full((h,w),-1,np.int8);a[prob>m['occupied_thresh']]=100;a[prob<m['free_thresh']]=0
 return a,m

def resample(a,m,alignment,points):
 assert alignment['scale']==1 and alignment['direction']=='estimated map coordinates -> gazebo_world'
 t=alignment['theta_rad'];c,s=math.cos(t),math.sin(t);r=np.array([[c,-s],[s,c]]);xy=(points-[alignment['tx_m'],alignment['ty_m']])@r
 ox,oy,theta=m['origin'];c,s=math.cos(theta),math.sin(theta);local=(xy-[ox,oy])@np.array([[c,-s],[s,c]])/m['resolution'];ij=np.floor(local).astype(np.int64)
 valid=(ij[:,0]>=0)&(ij[:,0]<a.shape[1])&(ij[:,1]>=0)&(ij[:,1]<a.shape[0]);out=np.full(len(points),-1,np.int8);out[valid]=a[ij[valid,1],ij[valid,0]]
 return out,ij,valid

def boundary(occupied,domain):
 # Four-neighbor inner occupied boundary; outside-domain neighbors ignored.
 nonoccupied=domain&~occupied;adj=np.zeros(domain.shape,bool)
 adj[1:,:]|=nonoccupied[:-1,:];adj[:-1,:]|=nonoccupied[1:,:];adj[:,1:]|=nonoccupied[:,:-1];adj[:,:-1]|=nonoccupied[:,1:]
 return occupied&domain&adj

def confusion(truth,pred):
 tp=int((truth&pred).sum());fp=int((~truth&pred).sum());fn=int((truth&~pred).sum());tn=int((~truth&~pred).sum())
 def ratio(a,b):return a/b if b else None
 return {'tp':tp,'fp':fp,'fn':fn,'tn':tn,'occupied_precision':ratio(tp,tp+fp),'occupied_recall':ratio(tp,tp+fn),'occupied_f1':ratio(2*tp,2*tp+fp+fn),'occupied_iou':ratio(tp,tp+fp+fn)}

def aggregate(values,seed):
 a=np.array([x for x in values if x is not None],float)
 if not len(a):return {'n':0,'mean':None,'sd':None,'median':None,'iqr':None,'min':None,'max':None,'bootstrap_mean_95_ci':None}
 rng=np.random.default_rng(seed);means=rng.choice(a,size=(20000,len(a)),replace=True).mean(axis=1);q=np.quantile(a,[.25,.75])
 return {'n':len(a),'mean':float(a.mean()),'sd':float(a.std(ddof=1)) if len(a)>1 else None,'median':float(np.median(a)),'q1':float(q[0]),'q3':float(q[1]),'iqr':float(q[1]-q[0]),'min':float(a.min()),'max':float(a.max()),'bootstrap_mean_95_ci':np.quantile(means,[.025,.975]).tolist()}

def main(output):
 output=Path(output);assert not output.exists(),'Refusing overwrite';assert sha(REF/'occupancy.npy')==HASH
 metadata=json.loads((REF/'generation_metadata.json').read_text());assert metadata['generator_git_commit'].startswith('09a8aea') and metadata['reference_status']=='VALIDATED_READY_TO_FREEZE'
 # Freeze all reference-generator source files at the approved commit.
 for p in (WS/'experiments/reference_v0_r1').iterdir():
  if p.is_file():assert p.read_bytes()==subprocess.check_output(['git','-C',str(WS),'show','09a8aea:'+str(p.relative_to(WS))])
 output.mkdir(parents=True);truth=np.load(REF/'occupancy.npy');refmap,rm=load_map(REF/'ideal_observed_map.yaml');assert np.array_equal(truth,refmap)
 domain=truth!=-1;assert int(domain.sum())==8649
 y,x=np.indices(truth.shape);allpoints=np.stack([rm['origin'][0]+(x+.5)*rm['resolution'],rm['origin'][1]+(y+.5)*rm['resolution']],axis=-1);points=allpoints[domain];tb=boundary(truth==100,domain);tp=allpoints[tb]
 np.save(output/'reference_observed_domain.npy',domain);np.save(output/'reference_boundary.npy',tb)
 inputs={};rows=[]
 protected=json.loads((ROOT/'baseline_run_004_013/source_output_sha256.json').read_text())
 assert all(sha(ROOT/f)==h for f,h in protected.items())
 for run in [f'run_{i:03d}' for i in range(4,14)]:
  src=ROOT/run;dst=output/run;dst.mkdir();a,m=load_map(src/'map.yaml');runmeta=json.loads((src/'run_metadata.json').read_text());alignment=runmeta['trajectory_evaluation']['alignment'];pred,ij,inside=resample(a,m,alignment,points);out=np.full(truth.shape,-1,np.int8);out[domain]=pred
  c=confusion(truth[domain]==100,pred==100);pb=boundary(out==100,domain);pp=allpoints[pb]
  status='ok' if len(pp) and len(tp) else 'empty_boundary'
  distances=[]
  for name,source,target in [('reference_to_slam',tp,pp),('slam_to_reference',pp,tp)]:
   if len(target) and len(source):d,nearest=cKDTree(target).query(source);distances.extend(d.tolist());audit=np.c_[source,target[nearest],d]
   else:audit=np.empty((0,5))
   np.savetxt(dst/(name+'.csv'),audit,delimiter=',',header='source_x,source_y,nearest_x,nearest_y,distance_m',comments='',fmt='%.17g')
  d=np.array(distances);metrics={k:c[k] for k in METRICS[:4]};metrics.update(boundary_distance_mean_m=float(d.mean()) if status=='ok' else None,boundary_distance_median_m=float(np.median(d)) if status=='ok' else None,boundary_distance_p95_m=float(np.quantile(d,.95)) if status=='ok' else None)
  report={'run_id':run,'status':status,'original_slam_success':runmeta['success'],'alignment':alignment,'reference_observed_cells':int(domain.sum()),'slam_known_in_domain':int((pred!=-1).sum()),'slam_unknown_in_domain':int((pred==-1).sum()),'outside_slam_extent_in_domain':int((~inside).sum()),'reference_boundary_cells':len(tp),'slam_boundary_cells':len(pp),'confusion':{k:c[k] for k in ['tp','fp','fn','tn']},**metrics}
  np.save(dst/'resampled_slam.npy',out);np.save(dst/'slam_boundary.npy',pb)
  category=np.where(truth[domain]==100,np.where(pred==100,1,3),np.where(pred==100,2,4))
  np.savetxt(dst/'cells.csv',np.c_[x[domain],y[domain],points,ij,inside.astype(int),truth[domain],pred,category],delimiter=',',header='reference_ix,reference_iy,world_x,world_y,source_ix,source_iy,in_source_extent,reference_value,slam_value,category_1TP_2FP_3FN_4TN',comments='',fmt=['%d','%d','%.17g','%.17g','%d','%d','%d','%d','%d','%d'])
  (dst/'metrics.json').write_text(json.dumps(report,indent=2));rows.append(report)
  for f in ['map.pgm','map.yaml','run_metadata.json']:inputs[str(src/f)]=sha(src/f)
 inputs[str(REF/'occupancy.npy')]=HASH;inputs[str(REF/'ideal_observed_map.yaml')]=sha(REF/'ideal_observed_map.yaml');inputs[str(REF/'ideal_observed_map.pgm')]=sha(REF/'ideal_observed_map.pgm')
 report={'reference_generator_commit':metadata['generator_git_commit'],'reference_occupancy_sha256':HASH,'scoring_git_commit':subprocess.check_output(['git','-C',str(WS),'rev-parse','HEAD'],text=True).strip(),'included_runs':[r['run_id'] for r in rows],'individual_runs':rows,'metrics':{k:{'individual_values':[r[k] for r in rows],**aggregate([r[k] for r in rows],20260915+i)} for i,k in enumerate(METRICS)},'input_sha256':inputs,'method':{'alignment':'Reuse each run_metadata trajectory_evaluation alignment verbatim; invert to sample source map. No fitted map alignment.','sampling':'Reference cell centers inverse-projected into SLAM map and YAML origin yaw; containing source cell (floor), no smoothing or tolerance dilation.','unknown':'Reference unknown excluded. Source unknown/out-of-map counts as not occupied: FN for occupied truth, TN for free truth. Known coverage reported separately.','boundary':'4-neighbor inner occupied boundary with at least one reference-observed nonoccupied neighbor. Outside-domain neighbors ignored. On the SLAM mask, unknown inside domain is nonoccupied.','distance':'Euclidean distances between boundary-cell centers on reference grid, meters. Concatenate both directed nearest-neighbor arrays; mean/median/p95 over pooled points. Point-count weighted, not equal-direction weighting.','statistics':'One value per run. Sample SD ddof=1; NumPy linear quantiles; IQR=Q3-Q1. 20,000 bootstrap resamples of runs; percentile95 CI for mean; seeds20260915+metric_index. Null metrics omitted explicitly, available n reported.','ambiguities':['Grid-cell-center sampling is phase-dependent at 5 cm; not overlap-area scoring.','Boundary metrics use cell centers and are quantized; excluded-domain neighbors never create artificial crop boundaries.','Unknown source cells are not silently removed from scoring.','Symmetric distance uses pooled boundary points; an equal-direction Chamfer mean is a different convention.','Reference omits the first canonical scan, while saved SLAM maps retain their original startup behavior.']}}
 (output/'report.json').write_text(json.dumps(report,indent=2));
 with (output/'per_run.csv').open('w') as f:
  fields=['run_id','status','slam_known_in_domain','slam_unknown_in_domain','outside_slam_extent_in_domain']+METRICS;w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
 lines=['# V0/R1 offline map scores','', '| Metric | Mean ± sample SD | Median | IQR | Min–max | Bootstrap 95% CI (mean) |','|---|---:|---:|---:|---:|---:|']
 for k,v in report['metrics'].items():
  ci=v['bootstrap_mean_95_ci'];lines.append(f"| {k} | {v['mean']:.6g} ± {v['sd']:.6g} | {v['median']:.6g} | {v['iqr']:.6g} | {v['min']:.6g}–{v['max']:.6g} | {ci[0]:.6g}–{ci[1]:.6g} |")
 lines+=['','All ten individual values are in per_run.csv/report.json. Per-cell assignments and','both directed boundary distances are retained per run. See report.json for exact','resampling, unknown-space, boundary and bootstrap definitions. No SLAM reruns.'];(output/'SUMMARY.md').write_text('\n'.join(lines)+'\n')
 assert all(sha(ROOT/f)==h for f,h in protected.items());assert all(sha(Path(f))==h for f,h in inputs.items());print(json.dumps(report['metrics'],indent=2))
if __name__=='__main__':p=argparse.ArgumentParser();p.add_argument('output');main(p.parse_args().output)
