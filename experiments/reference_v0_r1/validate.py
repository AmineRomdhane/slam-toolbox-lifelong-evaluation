"""Reference-only visibility, pose, and geometry validation; never reads SLAM maps."""
import json,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib.path import Path as Polygon
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def validate(folder):
 p=Path(folder);m=json.loads((p/'grid_metadata.json').read_text());a=np.load(p/'occupancy.npy');free=np.load(p/'free_observations.npy');hits=np.load(p/'hit_observations.npy');end=np.load(p/'ray_endpoints_xyz.npy');r=np.genfromtxt(p/'rays.csv',delimiter=',',names=True);g=np.genfromtxt(p/'robot_poses.csv',delimiter=',',names=True);l=np.genfromtxt(p/'lidar_poses.csv',delimiter=',',names=True);inter=json.loads((p/'interpolation.json').read_text())
 origin=np.array(m['origin_xyz'][:2]);res=m['resolution_m'];yy,xx=np.indices(a.shape);centers=np.stack([origin[0]+(xx+.5)*res,origin[1]+(yy+.5)*res],axis=-1);finite=np.isfinite(r['range_m'])&(r['range_m']<3.5);he=end[finite]
 checks={'scan_counts':inter['counts']=={'exact':49,'interpolated':805,'omitted_no_bracket':1} and len(l)==854,'no_extrapolation':not inter['extrapolation'],'unknown_has_no_observations':bool(np.all((a==-1)==((free==0)&(hits==0)))),'occupied_has_genuine_hit':bool(np.all((a==100)==(hits>0))),'free_has_traversal':bool(np.all((a==0)==((free>0)&(hits==0)))),'range_limits':bool(np.all(r['range_m']>=.12)),'ray_count':len(r)==307440}
 robot=np.stack([g[k] for k in ['x','y','z']],1);lidar=np.stack([l[k] for k in ['x','y','z']],1);q=np.stack([g[k] for k in ['qx','qy','qz','qw']],1);expected=robot+Rotation.from_quat(q).apply(np.tile([-.032,0,.171],(len(g),1)));error=np.linalg.norm(lidar-expected,axis=1);checks['physical_extrinsic_composition']=bool(error.max()<1e-12)
 post=[]
 for x in [-1.1,0,1.1]:
  for y in [-1.1,0,1.1]:
   radius=np.linalg.norm(he[:,:2]-[x,y],axis=1);on=np.abs(radius-.15)<.01;ix,iy=np.floor((np.array([x,y])-origin)/res).astype(int)
   post.append({'center_xy':[x,y],'expected_radius_m':.15,'first_hit_endpoints_within_1cm_of_surface':int(on.sum()),'center_cell':int(a[iy,ix]),'median_radial_error_m':float(np.median(np.abs(radius[on]-.15))) if on.any() else None})
 checks['post_centers_occluded_unknown']=all(v['center_cell']==-1 for v in post)
 checks['near_route_posts_observed']=all(v['first_hit_endpoints_within_1cm_of_surface']>0 for v in post if v['center_xy'][0]==-1.1)
 # Known body mesh inner/outer wall coordinates, for diagnostic validation only.
 mesh=Path('/home/roam5170/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_world/meshes/wall.dae');ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};root=ET.parse(mesh);v=np.fromstring(root.find('.//c:geometry/c:mesh/c:source/c:float_array',ns).text,sep=' ').reshape(-1,3);unit=float(root.find('.//c:unit',ns).attrib['meter']);rot=Rotation.from_euler('z',-1.5708)
 inner=rot.apply(np.c_[v[:24:4,:2]*unit*.25,np.zeros(6)])[:,:2];outer=rot.apply(np.c_[v[24:48:4,:2]*unit*.25,np.zeros(6)])[:,:2]
 # Order perimeter vertices before polygon membership.
 outer=outer[np.argsort(np.arctan2(outer[:,1],outer[:,0]))];outside=~Polygon(outer).contains_points(centers.reshape(-1,2)).reshape(a.shape)
 checks['outside_enclosing_wall_unknown']=bool(np.all(a[outside]==-1))
 walls=[]
 for i in range(6):
  b=inner[i];c=inner[(i+1)%6];d=c-b;t=np.clip((he[:,:2]-b)@d/(d@d),0,1);dist=np.linalg.norm(he[:,:2]-(b+t[:,None]*d),axis=1);mask=dist<.01;walls.append({'start_xy':b.tolist(),'end_xy':c.tolist(),'first_hit_endpoints_within_1cm':int(mask.sum()),'median_distance_m':float(np.median(dist[mask])) if mask.any() else None})
 checks['six_inner_wall_faces_observed']=all(w['first_hit_endpoints_within_1cm']>0 for w in walls)
 report={'checks':checks,'passed':all(checks.values()),'post_geometry':post,'body_wall_geometry':walls,'outside_wall_unknown_cells':int(outside.sum()),'physical_extrinsic_max_error_m':float(error.max()),'max_lidar_step_m':float(np.linalg.norm(np.diff(lidar,axis=0),axis=1).max()),'max_robot_step_m':float(np.linalg.norm(np.diff(robot,axis=0),axis=1).max()),'max_orientation_step_rad':float((Rotation.from_quat(q[:-1]).inv()*Rotation.from_quat(q[1:])).magnitude().max()),'visibility_scope':'Unknown means never traversed/hit by any accepted ray. A cell behind one hit may legitimately be observed from another pose. Post centers and outside enclosing wall independently tested.','mesh_diagnostic':'Inner/outer wall vertices read from original Collada position array, unit .0254 and SDF scale .25/yaw -1.5708; diagnostic only, never used to cast or fill grid.'}
 (p/'validation.json').write_text(json.dumps(report,indent=2))
 fig,ax=plt.subplots(figsize=(9,8));image=np.full(a.shape,.65);image[a==0]=1;image[a==100]=0;ax.imshow(image,origin='lower',extent=[origin[0],origin[0]+a.shape[1]*res,origin[1],origin[1]+a.shape[0]*res],cmap='gray',vmin=0,vmax=1,interpolation='nearest');ax.plot(g['x'],g['y'],color='#0072B2',lw=1.8,label='Canonical GT R1');ax.scatter(g['x'][0],g['y'][0],c='#D55E00',s=40,label='First included scan pose');ax.set(xlabel='Gazebo world x (m)',ylabel='Gazebo world y (m)',title='V0 / R1 ideal observed reference — 854 scans, 0.05 m cells');ax.legend();ax.set_aspect('equal');fig.tight_layout();fig.savefig(p/'reference_with_gt.png',dpi=180);plt.close(fig)
 return report
if __name__=='__main__':import sys;print(json.dumps(validate(sys.argv[1]),indent=2))
