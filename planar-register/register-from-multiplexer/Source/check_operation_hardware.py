"""Narrow-phase audit of every printed/native candidate in the operation sweep.
Surface sampling is necessary because inherited LEGO meshes are not closed solids.
"""
from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
s=(R/'Source/full_contact_inventory.py').read_text();exec(s[:s.index('records={}')])
P={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in ps};M={p['id']:p for p in meta};H={}
for h in hs:
 tri=v[h['offset']//3:h['offset']//3+h['vertices']].reshape(-1,3,3)
 H[h['id']]=np.unique(np.concatenate([tri.reshape(-1,3),tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]),axis=0)
rows=[];native=[]
for i,r in enumerate(json.loads((O/'Operation checks.json').read_text())['native_contact_candidates']):
 a,b=r['pair']
 if a not in P and b in P:a,b=b,a
 if a not in P:native.append(r);continue
 t=P[a];bounds=t.bounds;best=0;at=None
 for qm,qe in r['samples']:
  pts=trimesh.transform_points(H[b],np.linalg.inv(pose(M[a],qm,qe))@pose(M[b],qm,qe));pts=pts[np.all((pts>=bounds[0]-.001)&(pts<=bounds[1]+.001),axis=1)]
  if len(pts):
   dist=trimesh.proximity.signed_distance(t,pts)
   if not np.all(np.isfinite(dist)):raise RuntimeError(('Non-finite signed distance',a,b,qm,qe))
   depth=float(dist.max())
   if depth>best:best=depth;at=[qm,qe]
 rows.append(dict(pair=[a,b],max_sampled_surface_interior_mm=best,sample=at,relative_poses=len(r['samples'])))
 if i%10==0:print('Pair',i,'max depth',best,flush=True)
(O/'Operation hardware checks.json').write_text(json.dumps(dict(printed_native_checks=rows,native_native_unresolved=native,method='Native vertices, face centroids and edge midpoints tested against closed printed solids. Not a continuous or volumetric native-native proof.'),indent=2))
print(json.dumps([r for r in rows if r['max_sampled_surface_interior_mm']>.17],indent=2))
