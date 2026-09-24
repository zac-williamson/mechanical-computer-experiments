"""Native clutch surface crossings in every recorded phase-model pose.

Checks actual triangular surfaces, including the shallow entry chamfers.
Surface contact does not establish force, friction, or impact performance.
"""
from pathlib import Path
import json,hashlib,sys
import numpy as np,trimesh
from compact_pose import transform
R=Path(__file__).resolve().parents[1]/'Compact layout';ps={p['id']:p for p in json.loads((R/'parts.json').read_text())};v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
dense='--dense' in sys.argv
r=json.loads((R/('Compact phase-driven operation.json' if dense else 'Compact contact-resolved operation.json')).read_text());hits=[];checks=0;seen=set();pairs=[]
if dense:
 from compact_phase_operation import run
 cases=(run(c['start'],c['end'],c['initial_Q'],keep=True,record_stride=1) for c in r['cases'])
else:cases=r['cases']
for bank in ['master','slave','write','master_gate','slave_gate']:
 for gear in (['L102'] if '_gate' in bank else ['L072','L102']):
  ns=[bank+' L099',bank+' '+gear];cm=trimesh.collision.CollisionManager()
  for n in ns:
   p=ps[n];a=v[p['offset']//3:p['offset']//3+p['vertices']];cm.add_object(n,trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True))
  pairs.append((ns,cm))
for ci,c in enumerate(cases):
 for fi,f in enumerate(c['frames']):
  for ns,cm in pairs:
   ts=[transform(ps[n],f) for n in ns];relative=np.linalg.inv(ts[1])@ts[0];key=(ns[0],ns[1],tuple(np.round(relative[:3].ravel(),6)))
   if key in seen:continue
   seen.add(key);checks+=1
   for n,t in zip(ns,ts):cm.set_transform(n,t)
   col,names,data=cm.in_collision_internal(return_names=True,return_data=True)
   if col:hits.append(dict(case=ci,frame=fi,pair=ns,contacts=len(data)))
 if ci%14==0:print(ci,checks,len(hits),flush=True)
out=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),checks=checks,crossing_poses=len(hits),examples=hits[:100],sampled_dog_surface_pass=not hits,mechanically_qualified=False)
(R/('Dense native dog surface checks.json' if dense else 'Native dog surface checks.json')).write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
