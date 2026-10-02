"""All printed pairs in recorded contact-solved operation poses.

Includes levers, actual circular cam lift and clock amplification. Finite
sampling only; no tolerance, native gear mesh, force or continuous proof.
"""
from pathlib import Path
import json,hashlib
import numpy as np
import trimesh,manifold3d as m
from compact_pose import transform
R=Path(__file__).resolve().parents[1]/'Compact layout'
geometry_digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
items=[]
for p in parts:
 if p['kind']!='printed':continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 mesh=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 s=m.Manifold(m.Mesh64(mesh.vertices.astype(float),mesh.faces.astype(np.uint64)))
 assert mesh.is_watertight and s.status()==m.Error.NoError,p['id']
 items.append((p,s))
report=json.loads((R/'Compact contact-resolved operation.json').read_text())
assert report.get('contact_pose_pass'), 'Unresolved contact poses'
assert report['geometry_sha256']==geometry_digest, 'Stale contact trace'
frames=[f for c in report['cases'] for f in c.get('frames',[])];hits={};checks=0;cache={};unique=set()
for fi,f in enumerate(frames):
 posed=[]
 matrices=[transform(p,f)[:3] for p,s in items]
 state_key=tuple(np.round(np.concatenate([t.ravel() for t in matrices]),6))
 if state_key in unique:continue
 unique.add(state_key)
 for (p,s),matrix in zip(items,matrices):
  t=s.transform(matrix);posed.append((p,t,np.array(t.bounding_box()).reshape(2,3),tuple(np.round(matrix.ravel(),6))))
 for i,(p,s,b,pk) in enumerate(posed):
  for q,t,bb,qk in posed[i+1:]:
   if np.any(np.minimum(b[1],bb[1])-np.maximum(b[0],bb[0])<=0):continue
   key0=(p['id'],q['id'],pk,qk)
   if key0 in cache:vol=cache[key0]
   else:
    checks+=1;vol=(s^t).volume();cache[key0]=vol
   if vol>.001:
    key=(p['id'],q['id'])
    if key not in hits or hits[key]['volume_mm3']<vol:hits[key]=dict(a=key[0],b=key[1],volume_mm3=vol,frame=fi,turns=f['turns'])
result=dict(scope=__doc__,geometry_sha256=geometry_digest,frames=len(frames),unique_poses=len(unique),narrow_checks=checks,intersections=sorted(hits.values(),key=lambda x:-x['volume_mm3']),sampled_printed_motion_pass=not hits,mechanically_qualified=False)
(R/'Contact-solved printed motion.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
