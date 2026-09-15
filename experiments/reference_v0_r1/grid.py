"""Deterministic 2D traversal of Gazebo first-hit rays; no geometric ray caster."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
RES=.05

def cells_on_segment(start,end):
 """Half-open unit cells; corner crossings advance both axes, no zero-area side cells."""
 x,y=map(math.floor,start);ex,ey=map(math.floor,end);out=[(x,y)];dx,dy=end[0]-start[0],end[1]-start[1];sx=1 if dx>0 else -1 if dx<0 else 0;sy=1 if dy>0 else -1 if dy<0 else 0
 tx=((x+1 if sx>0 else x)-start[0])/dx if dx else math.inf;ty=((y+1 if sy>0 else y)-start[1])/dy if dy else math.inf
 dtx=abs(1/dx) if dx else math.inf;dty=abs(1/dy) if dy else math.inf
 for _ in range(abs(ex-x)+abs(ey-y)+3):
  if (x,y)==(ex,ey):return out
  if abs(tx-ty)<1e-12:x+=sx;y+=sy;tx+=dtx;ty+=dty
  elif tx<ty:x+=sx;tx+=dtx
  else:y+=sy;ty+=dty
  out.append((x,y))
 raise ValueError('Grid traversal did not converge')

def make_grid(folder,expected_scans=854):
 folder=Path(folder);r=np.genfromtxt(folder/'rays.csv',delimiter=',',names=True);poses=np.genfromtxt(folder/'lidar_poses.csv',delimiter=',',names=True)
 assert len(r)==expected_scans*360 and len(poses)==expected_scans
 assert np.array_equal(r['scan_index'],np.repeat(np.arange(expected_scans),360));assert np.array_equal(r['ray_index'],np.tile(np.arange(360),expected_scans));assert np.array_equal(r['canonical_stamp_ns'],np.repeat(poses['stamp_ns'],360))
 assert np.allclose(r['angle_rad'],np.tile(np.arange(360)*6.28/359,expected_scans),rtol=0,atol=1e-14)
 origin=np.floor((np.array([poses['x'].min(),poses['y'].min()])-3.5)/RES)*RES-RES
 top=np.ceil((np.array([poses['x'].max(),poses['y'].max()])+3.5)/RES)*RES+RES
 width,height=np.rint((top-origin)/RES).astype(int);free=np.zeros((height,width),np.uint32);occupied=free.copy();first_hit_ray=np.full((height,width),-1,np.int64)
 hit=np.isfinite(r['range_m']) & (r['range_m']<3.5);assert not np.isnan(r['range_m']).any() and np.all(r['range_m']>=.12)
 starts=np.stack([r[k] for k in ['world_x','world_y','world_z']],axis=1);quat=np.stack([r[k] for k in ['qx','qy','qz','qw']],axis=1);directions=Rotation.from_quat(quat).apply(np.stack([np.cos(r['angle_rad']),np.sin(r['angle_rad']),np.zeros(len(r))],axis=1));ends=starts+np.minimum(r['range_m'],3.5)[:,None]*directions
 for i,(s,e,is_hit) in enumerate(zip(starts,ends,hit)):
  cells=cells_on_segment((s[:2]-origin)/RES,(e[:2]-origin)/RES)
  for x,y in cells[:-1] if is_hit else cells:
   assert 0<=x<width and 0<=y<height;free[y,x]+=1
  if is_hit:
   x,y=cells[-1];occupied[y,x]+=1
   if first_hit_ray[y,x]<0:first_hit_ray[y,x]=i
 grid=np.full((height,width),-1,np.int8);grid[free>0]=0;grid[occupied>0]=100
 np.save(folder/'occupancy.npy',grid,allow_pickle=False)
 for name,array in [('free_observations',free),('hit_observations',occupied),('first_hit_ray',first_hit_ray),('ray_endpoints_xyz',ends)]:np.save(folder/(name+'.npy'),array,allow_pickle=False)
 pixels=np.full(grid.shape,205,np.uint8);pixels[grid==0]=254;pixels[grid==100]=0
 (folder/'ideal_observed_map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+np.flipud(pixels).tobytes())
 (folder/'ideal_observed_map.yaml').write_text(f'image: ideal_observed_map.pgm\nresolution: 0.05\norigin: [{origin[0]:.17g}, {origin[1]:.17g}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
 metadata={'frame':'gazebo_world','resolution_m':RES,'origin_xyz':[float(origin[0]),float(origin[1]),0.],'width':int(width),'height':int(height),'array_layout':'int8 [y,x], row 0 at minimum world y; numpy .npy uncompressed','cell_counts':{'free':int((grid==0).sum()),'occupied':int((grid==100).sum()),'unknown':int((grid==-1).sum()),'observed':int((grid!=-1).sum()),'total':int(grid.size)},'free_hit_conflict_cells':int(((free>0)&(occupied>0)).sum()),'scan_count':expected_scans,'ray_count':len(r),'finite_first_hits':int(hit.sum()),'no_hits_before_max_range':int((~hit).sum()),'occupancy_sha256':hashlib.sha256((folder/'occupancy.npy').read_bytes()).hexdigest(),'pgm_sha256':hashlib.sha256((folder/'ideal_observed_map.pgm').read_bytes()).hexdigest(),'conflict_rule':'Occupied iff at least one genuine finite first-hit endpoint falls in cell; else free iff traversed. Occupied wins free/hit discretization conflicts only.','traversal_rule':'Amanatides-Woo XY segment traversal, half-open cells. Exact corners advance both axes. Terminal hit cell occupied; no-hit terminal cell free. No traversal beyond first hit. No interpolation between angular beams.','near_clip_note':'As explicitly requested, free traversal begins at origin, including the 0–0.12 m blind segment. No observations inside min range are fabricated as hits.','projection':'Full SE(3) 3D rays, endpoints projected to XY; range limit is 3D ray distance.'}
 (folder/'grid_metadata.json').write_text(json.dumps(metadata,indent=2));return metadata
if __name__=='__main__':p=argparse.ArgumentParser();p.add_argument('folder');print(json.dumps(make_grid(p.parse_args().folder),indent=2))
