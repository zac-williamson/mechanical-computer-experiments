"""Candidate static printed-solid check. No broad gear/contact exemptions."""
from pathlib import Path
import hashlib
import json
import numpy as np
import trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Assembly development'
ps=[p for p in json.loads((O/'parts.json').read_text()) if p['kind']=='printed']
ss={};bs={}
for p in ps:
 t=trimesh.load(O/(p['id']+'.stl'))
 # Check fixed frame only, independently from actuator phase model.
 ss[p['id']]=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 bs[p['id']]=t.bounds
hits=[];pairs=0
for i,p in enumerate(ps):
 for q in ps[i+1:]:
  if p['motion']!='fixed' or q['motion']!='fixed':continue
  a,b=bs[p['id']],bs[q['id']]
  if np.any(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])<1e-6):continue
  pairs+=1;v=(ss[p['id']]^ss[q['id']]).volume()
  if v>1e-4:hits.append(dict(a=p['id'],b=q['id'],volume_mm3=v))
report=dict(scope='Fixed printed parts only; does not clear moving parts or native hardware',tested_broadphase_pairs=pairs,intersections=hits)
report['assembly_sha256']=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
(R/'Layout fixed-part checks.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
