"""Finite installation screen for a closed lock-return band over its free front lip.

Expanded loop approaches along +Y with lock bolt absent. Other printed parts
remain assembled at neutral; band thickness is 0.8 mm. This proves only this
specified geometric path, not a material stretch limit or hand/tool access.
"""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'Wall register';P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def cyl(r,lo,hi,c):return m.Manifold.cylinder(hi-lo,r,circular_segments=64).rotate([-90,0,0]).translate([c[0],lo,c[1]])
rows=[]
for bank,x in [('master',-5.05),('slave',100.95)]:
 scene=[]
 for p in P:
  if p['kind']!='printed' or p['module']!='bit' or p['id']==bank+' lock bolt':continue
  b=np.array(p['bounds']);
  if np.all(b[1]>[x-3.5,30,46.8]) and np.all(b[0]<[x+3.5,38,53.2]):scene.append((p['id'],solid(V[p['offset']//3:p['offset']//3+p['vertices']])))
 hits=[]
 # Expand to clear 2.4 mm retaining flange, then contract into 1.8 mm seat.
 poses=[(float(y),2.6) for y in np.linspace(33.5,37.3,40)]+[(37.3,float(r)) for r in np.linspace(2.6,1.8,20)]
 for y,r in poses:
  loop=cyl(r+.8,y-.4,y+.4,[x,50])-cyl(r,y-.41,y+.41,[x,50])
  for n,s in scene:
   v=(loop^s).volume()
   if v>.005:hits.append(dict(part=n,y=y,inner_radius=r,volume_mm3=v))
 rows.append(dict(bank=bank,poses=len(poses),hits=hits,pass_check=not hits))
r=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,anchors=rows,band_installation_pass=all(x['pass_check'] for x in rows))
(O/'Band installation checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
