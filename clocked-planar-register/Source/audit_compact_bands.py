"""Elastic loop geometry against printed solids over conservative lever ranges.
No elastic force, preload, installation or fatigue qualification.
"""
import json,hashlib
from pathlib import Path
import numpy as np,trimesh
from compact_elastic import actuator_band
from compact_pose import transform
R=Path(__file__).resolve().parents[1]/'Compact layout';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3);printed=[]
for p in parts:
 if p['kind']=='printed':
  a=v[p['offset']//3:p['offset']//3+p['vertices']];printed.append((p,trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)))
hits={};checks=0
for bank in ['master','slave','write','clock']:
 for b in np.linspace(-28.5,28.5,115):
  band=np.unique(actuator_band(bank,b),axis=0)
  for q in [-3.756,0,3.75]:
   f={k:dict(q=q if k==bank else 0,b=b if k==bank else 0,w=0,g=0) for k in ['master','slave','write','clock']}
   f.update(angles=dict(D=0,WRITE=0,CLK=0,POWER=0,M=0,Q=0,X=0),rail=0,master_lift=0,slave_lift=0,rm=0,ro=0,rw=0)
   for p,t in printed:
    # Transform band vertices into each printed component's local frame.
    a=trimesh.transform_points(band,np.linalg.inv(transform(p,f)))
    a=a[np.all((a>t.bounds[0])&(a<t.bounds[1]),axis=1)]
    if not len(a):continue
    checks+=1;a=a[t.contains(a)]
    if not len(a):continue
    _,d,_=t.nearest.on_surface(a)
    if max(d)>.03:
     key=(bank,p['id'])
     if key not in hits or hits[key]['depth_mm']<max(d):hits[key]=dict(band=bank,printed=p['id'],depth_mm=float(max(d)),lever_deg=float(b),carriage_mm=q)
result=dict(scope=__doc__,geometry_sha256=digest,checks=checks,hits=sorted(hits.values(),key=lambda h:-h['depth_mm']),mechanically_qualified=False)
(R/'Actuator band screening.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
