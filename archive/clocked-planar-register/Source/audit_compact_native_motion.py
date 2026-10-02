"""All native/native triangle contacts over the recorded operation trace.

No pair is excluded. This reports surface contacts, including intentional
keyed fits, shared boundaries and meshing teeth. It cannot turn all contacts
into either failures or exemptions without interface-specific verification.
"""
from pathlib import Path
import hashlib,json
import numpy as np,trimesh
from compact_pose import transform
R=Path(__file__).resolve().parents[1]/'Compact layout'
digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
ps=[p for p in json.loads((R/'parts.json').read_text()) if p['kind']=='native']
v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
trace=json.loads((R/'Compact contact-resolved operation.json').read_text())
assert trace['geometry_sha256']==digest and trace['contact_pose_pass']
manager=trimesh.collision.CollisionManager()
for p in ps:
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 manager.add_object(p['id'],trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True))
seen=set();hits={};frames=0
for ci,case in enumerate(trace['cases']):
 for fi,f in enumerate(case['frames']):
  frames+=1;ts=[transform(p,f) for p in ps]
  key=np.round(np.stack(ts),5).tobytes()
  if key in seen:continue
  seen.add(key)
  for p,t in zip(ps,ts):manager.set_transform(p['id'],t)
  collision,names=manager.in_collision_internal(return_names=True)
  for pair in names:
   pair=tuple(sorted(pair));row=hits.setdefault(pair,dict(a=pair[0],b=pair[1],contact_poses=0,first_case=ci,first_frame=fi))
   row['contact_poses']+=1
 if ci%14==0:print(ci,frames,len(seen),len(hits),flush=True)
r=dict(scope=__doc__,geometry_sha256=digest,recorded_frames=frames,unique_assembly_poses=len(seen),surface_contacts=list(hits.values()),mechanically_qualified=False)
(R/'Native motion surface screening.json').write_text(json.dumps(r,indent=2))
print('Complete:',frames,len(seen),'poses;',len(hits),'contact pairs',flush=True)
