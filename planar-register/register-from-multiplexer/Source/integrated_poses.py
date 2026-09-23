"""Whole-assembly contact inventory. No part-group exclusions; contacts are not silently passed."""
from pathlib import Path
import json,numpy as np,trimesh
from integrated_cam_math import cam_lift
R=Path(__file__).resolve().parents[1];O=R.parent/'work/integrated-cam-development';B=R.parent/'work/register-mux-reference/multiplexer';ps=json.loads((O/'printed-parts.json').read_text());hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];trace=json.loads((B/'Switching trace.json').read_text())['frames'];meta=ps+hs;cm=trimesh.collision.CollisionManager()
for p in ps:cm.add_object(p['id'],trimesh.load(O/(p['id']+'.stl')))
for h in hs:
 a=v[h['offset']//3:h['offset']//3+h['vertices']];cm.add_object(h['id'],trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False))
def pose(p,qm,qe):
 T=np.eye(4);bank=p.get('bank');q=qm if bank=='Memory' else qe;mo=p['motion'];shift=np.array([0,0,0]) if bank=='Memory' else np.array([-85.0,0,16]);f=min(trace,key=lambda f:abs(f['q']-q)) if mo in ['rocker','gear','worm'] else None
 if mo in ['carriage','worm']:T[0,3]=q
 if mo=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
 if mo=='rocker':T=trimesh.transformations.rotation_matrix(np.radians(f['b']),[0,1,0],np.array([13.192323604,10.2,32.128448698])+shift)
 a=0;up=cam_lift(qe)
 if abs(qm)<3.0:up=max(up,3.2)
 if mo=='bolt':T[2,3]=up
 if mo=='release-crank':T=trimesh.transformations.rotation_matrix(a,[0,1,0],[60,0,54])
 # Prescribed source reaction/worm phase, not an independent-force model.
 if mo=='gear':T=trimesh.transformations.rotation_matrix(np.radians(f['g']),[0,1,0],np.array([0,10.2,24])+shift)
 if mo=='worm':
  T=trimesh.transformations.rotation_matrix(np.radians(f['w']),[1,0,0],np.array([0,10.2,16])+shift);T[0,3]+=q
 return T
