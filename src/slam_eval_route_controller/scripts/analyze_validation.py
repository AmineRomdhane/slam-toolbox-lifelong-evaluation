"""Analyze saved GT trajectories; conservative geometric collision screening."""
import csv,json,math,itertools,bisect,sys
from pathlib import Path
ROOT=Path(sys.argv[1])
rows=[]; results=[]
for trial in sorted((p for p in ROOT.glob('run_[0-9]*') if p.is_dir()),key=lambda p:int(p.name.split('_')[1])):
    i=int(trial.name.split('_')[1])
    with (ROOT/f'run_{i}'/'trajectory.csv').open() as f:
        rr=list(csv.DictReader(f))
    rows.append(rr)
    results.append(json.loads((ROOT/f'run_{i}'/'result.json').read_text()))
def edges(poly):return list(zip(poly,poly[1:]+poly[:1]))
def dist(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
wall=[(2.932938,0),(1.466469,2.54),(-1.466469,2.54),(-2.932938,0),(-1.466469,-2.54),(1.466469,-2.54)]
def inside(p,poly):
    return all((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])>=-1e-9 for a,b in edges(poly))
h0=[(57.735,0),(28.8675,50),(-28.8675,50),(-57.735,0),(-28.8675,-50),(28.8675,-50)]
hexes=[[(x+a*.0254*s,y+b*.0254*s) for a,b in h0] for x,y,s in [(3.5,0,.8),(1.8,2.7,.55),(1.8,-2.7,.55),(-1.8,2.7,.55),(-1.8,-2.7,.55)]]
trajectories=[]
for rr,res in zip(rows,results):
    xy=[(float(r['x']),float(r['y'])) for r in rr]
    times=[int(r['sec'])+int(r['nanosec'])*1e-9 for r in rr]
    times=[t-times[0] for t in times]
    arc=[0.]
    for a,b in zip(xy,xy[1:]):arc.append(arc[-1]+math.dist(a,b))
    clearance=1e9
    for p in xy:
        w=min(dist(p,a,b) for a,b in edges(wall))
        if not inside(p,wall):w=-w
        clearance=min(clearance,w-.2)
        for x in [-1.1,0,1.1]:
            for y in [-1.1,0,1.1]:clearance=min(clearance,math.dist(p,(x,y))-.15-.2)
        for h in hexes:
            d=min(dist(p,a,b) for a,b in edges(h))
            clearance=min(clearance,(-d if inside(p,h) else d)-.2)
    maxstep=max(math.dist(a,b) for a,b in zip(xy,xy[1:]))
    res['min_conservative_clearance_m']=clearance
    res['continuous_clearance_lower_bound_m']=clearance-maxstep
    res['geometric_collision']=clearance-maxstep<=0
    res['physics_contact_status']='not instrumented'
    res['max_commanded_linear_acceleration']=max(abs(float(b['v'])-float(a['v']))/(tb-ta) for a,b,ta,tb in zip(rr,rr[1:],times,times[1:]) if tb>ta)
    res['max_commanded_angular_acceleration']=max(abs(float(b['w'])-float(a['w']))/(tb-ta) for a,b,ta,tb in zip(rr,rr[1:],times,times[1:]) if tb>ta)
    trajectories.append((times,[a/arc[-1] for a in arc],xy))
def interp(axis,xy,t):
    i=max(0,min(len(axis)-2,bisect.bisect_right(axis,t)-1))
    a=0 if axis[i+1]==axis[i] else (t-axis[i])/(axis[i+1]-axis[i])
    return tuple(xy[i][j]+a*(xy[i+1][j]-xy[i][j]) for j in [0,1])
pairs=[]
common=min(t[0][-1] for t in trajectories)
for i,j in itertools.combinations(range(len(rows)),2):
    entry={'runs':[i+1,j+1]}
    for mode,axis,extent in [('elapsed_time',0,common),('normalized_arc_length',1,1.)]:
        ds=[math.dist(interp(trajectories[i][axis],trajectories[i][2],extent*k/1000),
                      interp(trajectories[j][axis],trajectories[j][2],extent*k/1000)) for k in range(1001)]
        entry[mode]={'rms_m':math.sqrt(sum(d*d for d in ds)/len(ds)),'max_m':max(ds)}
    pairs.append(entry)
report={'runs':results,'pairwise_xy_repeatability':pairs,'common_time_seconds':common,
        'alignment':'Same Gazebo world frame; no spatial registration. 1001 interpolated samples per comparison.',
        'collision_method':'0.20 m swept circular envelope against static posts, wall, hexagons; no contact sensor.',
        'notes':'Rate/trajectory statistics include corner rotations. Tracking error is distance to the active finite nominal segment.'}
(ROOT/'summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
