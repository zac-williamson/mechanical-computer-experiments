"""Native vertex penetration screening. Does not clear edge-only intersections."""
from pathlib import Path
import hashlib
import json
import numpy as np,trimesh
from pose import transform
R=Path(__file__).resolve().parents[1];O=R/'Assembly development'
ps=json.loads((O/'parts.json').read_text());v=np.load(O/'geometry.npz')['vertices']
frames=next(c['frames'] for c in json.loads((R/'Angle-driven operation.json').read_text())['cases'] if c.get('frames'))
prints={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in ps if p['kind']=='printed'}
verts={p['id']:np.unique(v[p['offset']//3:p['offset']//3+p['vertices']],axis=0) for p in ps if p['kind']=='native'}
hits={}
for fi in [0,100,200,300,400]:
 f=frames[fi];Ts={p['id']:transform(p,f) for p in ps}
 for n,vs in verts.items():
  world=trimesh.transform_points(vs,Ts[n])
  for pn,t in prints.items():
   local=trimesh.transform_points(world,np.linalg.inv(Ts[pn]));inside=np.all((local>t.bounds[0]+.025)&(local<t.bounds[1]-.025),axis=1)
   a=local[inside]
   if not len(a):continue
   contained=t.contains(a)
   if not np.any(contained):continue
   a=a[contained];_,dist,_=t.nearest.on_surface(a);depth=float(dist.max())
   if depth>.03:
    key=(n,pn)
    if key not in hits or hits[key]['depth_mm']<depth:hits[key]=dict(native=n,printed=pn,depth_mm=depth,frame=fi,point_local=a[dist.argmax()].tolist())
 print('frame',fi,'penetrating pairs',len(hits),flush=True)
report=dict(scope='Five sampled poses, native vertices inside printed solids; edge-only intersections not tested',intersections=sorted(hits.values(),key=lambda h:-h['depth_mm']))
report['assembly_sha256']=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
(R/'Native penetration checks.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
