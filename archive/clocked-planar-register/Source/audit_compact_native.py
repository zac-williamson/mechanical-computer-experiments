"""Native vertices inside printed solids at the reference pose.

Includes every native/printed pair. Does not clear edge-only penetration,
tooth phasing, intended pin interference fits, or any moving pose.
"""
from pathlib import Path
import hashlib,json
import numpy as np
import trimesh
R=Path(__file__).resolve().parents[1]/'Compact layout'
geometry_digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((R/'parts.json').read_text())
v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
prints={};natives={}
for p in parts:
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 if p['kind']=='printed':
  prints[p['id']]=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 else:natives[p['id']]=np.unique(a,axis=0)
hits=[]
for name,a in natives.items():
 for pn,t in prints.items():
  inside=np.all((a>t.bounds[0]+.025)&(a<t.bounds[1]-.025),axis=1)
  b=a[inside]
  if not len(b):continue
  b=b[t.contains(b)]
  if not len(b):continue
  _,dist,_=t.nearest.on_surface(b)
  if dist.max()>.03:
   hits.append(dict(native=name,printed=pn,depth_mm=float(dist.max()),point=b[dist.argmax()].tolist()))
report=dict(scope=__doc__,geometry_sha256=geometry_digest,
 intersections=sorted(hits,key=lambda p:-p['depth_mm']),operation_pass=False)
(R/'Native vertex screening.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
