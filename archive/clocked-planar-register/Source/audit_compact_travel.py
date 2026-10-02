"""Printed support/travel screening. Does not validate gears, forces or timing.

Rockers require contact-solved poses and are explicitly outside this screen.
Clock transitions assume storage carriages have settled at their endpoints.
"""
from pathlib import Path
import hashlib, itertools, json, math
import numpy as np
import trimesh
import manifold3d as m
from compact_cam import lifts

root=Path(__file__).resolve().parents[1]/'Compact layout'
geometry_digest=hashlib.sha256((root/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((root/'parts.json').read_text())
vertices=np.load(root/'geometry.npz')['vertices'].reshape(-1,3)
items=[]; omitted=[]
for p in parts:
 if p['kind']!='printed':continue
 if p.get('motion') in ['rocker','lever']:
  omitted.append(p['id']);continue
 a=vertices[p['offset']//3:p['offset']//3+p['vertices']]
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 s=m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
 assert t.is_watertight and s.status()==m.Error.NoError,p['id']
 items.append((p,s))

def placed(p,s,c,w,master,slave):
 motion=p.get('motion','fixed');name=p['id']
 if motion=='carriage':
  x={'clock':c,'write':w,'master':master,'slave':slave}[name.split()[0]]
  return s.translate([x,0,0])
 if motion in ['crosshead','gate-fork']:return s.translate([2.5*c,0,0])
 if motion=='bolt':
  return s.translate([0,0,lifts(2.5*c)[name.split()[0]]])
 if motion=='amplifier':
  # Rotation around Y: x=r*sin(theta), z=r*cos(theta).
  return s.translate([-40,0,56]).rotate([0,math.degrees(math.asin(c/12)),0]).translate([40,0,-56])
 return s

states=[]
for c,w,a,b in itertools.product(np.linspace(-3.75,3.75,41),[-3.75,0,3.75],[-3.75,3.75],[-3.75,3.75]):
 states.append((float(c),w,a,b))
for c,w,q,held in itertools.product([-3.75,3.75],[-3.75,0,3.75],np.linspace(-3.75,3.75,41),[-3.75,3.75]):
 states.append((c,w,float(q) if c>0 else held,held if c>0 else float(q)))
hits={};narrow=0
for state in states:
 posed=[]
 for p,s in items:
  t=placed(p,s,*state)
  posed.append((p,t,np.array(t.bounding_box()).reshape(2,3)))
 for i,(p,a,ab) in enumerate(posed):
  for p2,b,bb in posed[i+1:]:
   if p.get('motion')=='fixed' and p2.get('motion')=='fixed':continue
   if np.any(np.minimum(ab[1],bb[1])-np.maximum(ab[0],bb[0])<=0):continue
   narrow+=1;volume=(a^b).volume()
   if volume>.001:
    key=(p['id'],p2['id'])
    if key not in hits or volume>hits[key]['volume_mm3']:
     hits[key]=dict(a=key[0],b=key[1],volume_mm3=volume,clock_write_master_slave_mm=state)
report=dict(scope=__doc__,geometry_sha256=geometry_digest,
 states=len(states),narrow_checks=narrow,omitted_rockers=omitted,
 intersections=sorted(hits.values(),key=lambda p:-p['volume_mm3']),
 sampled_support_travel_pass=not hits,operation_pass=False)
(root/'Printed travel screening.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
