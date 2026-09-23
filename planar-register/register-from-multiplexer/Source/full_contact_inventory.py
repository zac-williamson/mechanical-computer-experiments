"""Whole-assembly contact inventory. No part-group exclusions; contacts are not silently passed."""
from pathlib import Path
import json,numpy as np,trimesh
from direct_cam_math import cam_lift
R=Path(__file__).resolve().parents[1];O=R/'Planar register';B=R.parent/'work/register-mux-reference/multiplexer';ps=json.loads((O/'printed-parts.json').read_text());hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];trace=json.loads((B/'Switching trace.json').read_text())['frames'];meta=ps+hs;cm=trimesh.collision.CollisionManager()
for p in ps:cm.add_object(p['id'],trimesh.load(O/(p['id']+'.stl')))
for h in hs:
 a=v[h['offset']//3:h['offset']//3+h['vertices']];cm.add_object(h['id'],trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False))
def pose(p,qm,qe):
 T=np.eye(4);bank=p.get('bank');q=qm if bank=='Memory' else qe;mo=p['motion'];shift=np.array([0,0,0]) if bank=='Memory' else np.array([96.8,0,16]);f=min(trace,key=lambda f:abs(f['q']-q)) if mo in ['rocker','gear','worm'] else None
 if mo in ['carriage','worm']:T[0,3]=q
 if mo=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
 if mo=='rocker':T=trimesh.transformations.rotation_matrix(np.radians(f['b']),[0,1,0],np.array([13.192323604,10.2,32.128448698])+shift)
 a=0;up=cam_lift(qe)
 if abs(qm)<3.3:up=max(up,5.4)
 if mo=='bolt':T[2,3]=up
 if mo=='release-crank':T=trimesh.transformations.rotation_matrix(a,[0,1,0],[60,0,54])
 # Prescribed source reaction/worm phase, not an independent-force model.
 if mo=='gear':T=trimesh.transformations.rotation_matrix(np.radians(f['g']),[0,1,0],np.array([0,10.2,24])+shift)
 if mo=='worm':
  T=trimesh.transformations.rotation_matrix(np.radians(f['w']),[1,0,0],np.array([0,10.2,16])+shift);T[0,3]+=q
 return T
records={}
for qm in [-3.75,0,3.75]:
 for qe in [-3.75,0,3.75]:
  for p in meta:cm.set_transform(p['id'],pose(p,qm,qe))
  _,pairs,data=cm.in_collision_internal(return_names=True,return_data=True)
  for d in data:
   key=tuple(sorted(d.names));r=records.setdefault(key,dict(pair=key,max_depth=0,sample=[qm,qe],contact_point=d.point.tolist()))
   if d.depth>r['max_depth']:r.update(max_depth=float(d.depth),sample=[qm,qe],contact_point=d.point.tolist())
rows=sorted(records.values(),key=lambda r:-r['max_depth']);(O/'Full contact inventory.json').write_text(json.dumps(dict(poses=9,parts=len(meta),contacts=rows,scope='Every printed and modelled LEGO part included. Surface contacts require classification; nine poses are diagnostic, not full transition validation.'),indent=2));print(json.dumps(rows[:60],indent=2));print('Pairs',len(rows))
