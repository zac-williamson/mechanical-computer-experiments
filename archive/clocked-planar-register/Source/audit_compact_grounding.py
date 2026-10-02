"""Printed connection graph inferred from actual annular friction-pin bores.

This tests attachment geometry, not press-fit strength or rotational restraint.
A single pin does not qualify a structural joint; counts remain visible.
"""
import json,hashlib
from pathlib import Path
import numpy as np,trimesh
R=Path(__file__).resolve().parents[1]/'Compact layout';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
printed=[];pins=[]
for p in parts:
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 if p['kind']=='printed':printed.append((p,trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)))
 elif p.get('lego_part')=='2780' or 'Carriage support pin' in p['id']:pins.append((p,a))
links=[];graph={p['id']:set() for p,t in printed};attached={p['id']:[] for p,t in printed}
for pin,pa in pins:
 axis=pin.get('axis',int(np.argmax(np.ptp(pa,axis=0))));cross=[j for j in range(3) if j!=axis]
 c=np.array(pin.get('centre',(pa.min(0)+pa.max(0))/2),dtype=float);owners=[]
 for p,t in printed:
  if np.any(pa.max(0)<t.bounds[0]) or np.any(pa.min(0)>t.bounds[1]):continue
  valid=[]
  for depth in [-6,-4,-2,2,4,6]:
   pts=[]
   for radius in [2.,3.4]:
    for theta in np.linspace(0,2*np.pi,16,endpoint=False):
     point=c.copy();point[axis]+=depth;point[cross[0]]+=radius*np.cos(theta);point[cross[1]]+=radius*np.sin(theta);pts.append(point)
   inside=t.contains(pts)
   if not inside[:16].any() and inside[16:].sum()>=(6 if p['id'].startswith('Compact frame') else 12):valid.append(depth)
  if len(valid)>=2:owners.append(p['id']);attached[p['id']].append(pin['id'])
 for owner in owners:graph[owner].update(set(owners)-{owner})
 links.append(dict(pin=pin['id'],parts=owners))
roots={name for name in graph if name.startswith('Compact frame')};ground=set(roots);todo=list(roots)
while todo:
 for n in graph[todo.pop()]-ground:ground.add(n);todo.append(n)
fixed=[p['id'] for p,t in printed if p.get('motion','fixed')=='fixed']
result=dict(scope=__doc__,geometry_sha256=digest,pin_connections=links,fixed_parts_without_pin_path_to_frame=sorted(set(fixed)-ground),printed_pin_counts={n:len(x) for n,x in attached.items()},mechanically_qualified=False)
(R/'Grounding screening.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
