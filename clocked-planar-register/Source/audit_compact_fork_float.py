"""Printed fork clearance over allowed one-sided lost motion.

Sampled solid intersections, including full float at either end of the bar.
This does not establish band force, dog torque or continuous clearance.
"""
from pathlib import Path
import json,hashlib,itertools,math
import numpy as np,trimesh,manifold3d as m
from compact_cam import lifts
R=Path(__file__).resolve().parents[1]/'Compact layout'
digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest();ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
items=[]
for p in ps:
 if p['kind']!='printed' or p.get('motion') in ['rocker','lever']:continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 s=m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)));assert s.status()==m.Error.NoError,p['id'];items.append((p,s))
def pose(p,s,bar,w,mm,ss,lag):
 mode=p.get('motion');bank=p.get('bank',p['id'].split()[0]);dx=0
 if mode=='carriage':dx={'clock':bar/2.5,'write':w,'master':mm,'slave':ss}[bank]
 elif mode=='crosshead':dx=bar
 elif mode=='gate-fork':dx=bar+lag[0 if bank=='master_gate' else 1]
 elif mode=='bolt':return s.translate([0,0,lifts(bar)[bank]])
 elif mode=='amplifier':return s.translate([-40,0,56]).rotate([0,math.degrees(math.asin(bar/30)),0]).translate([40,0,-56])
 return s.translate([dx,0,0])
hits={};checks=0;states=0
for bar,w,mm,ss in itertools.product(np.linspace(-9.39,9.39,41),[-3.76,0,3.76],[-3.76,3.76],[-3.76,3.76]):
 for lag in [(0,0),(-2,0),(-4,0),(0,2),(0,4)]:
  # A waiting fork exists only on the engaging side; include full stop travel.
  if (lag[0] and bar<6.8) or (lag[1] and bar>-6.8):continue
  states+=1;posed=[]
  for p,s in items:
   a=pose(p,s,bar,w,mm,ss,lag);posed.append((p,a,np.array(a.bounding_box()).reshape(2,3)))
  for i,(p,a,b) in enumerate(posed):
   for q,c,d in posed[i+1:]:
    if p.get('motion')!='gate-fork' and q.get('motion')!='gate-fork':continue
    if np.any(np.minimum(b[1],d[1])-np.maximum(b[0],d[0])<=0):continue
    checks+=1;vol=(a^c).volume()
    if vol>.001:
     key=(p['id'],q['id'])
     if key not in hits or hits[key]['volume_mm3']<vol:hits[key]=dict(a=key[0],b=key[1],volume_mm3=vol,state=[bar,w,mm,ss],lag=lag)
r=dict(scope=__doc__,geometry_sha256=digest,states=states,checks=checks,intersections=list(hits.values()),sampled_fork_float_pass=not hits,mechanically_qualified=False)
(R/'Fork float clearance.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
