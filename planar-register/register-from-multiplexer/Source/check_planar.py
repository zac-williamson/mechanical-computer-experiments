from pathlib import Path
import json,itertools,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());P={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in meta}
sol={n:m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64))) for n,t in P.items()};cm=trimesh.collision.CollisionManager()
for n,t in P.items():cm.add_object(n,t)
changed={'Memory — Right carriage bearing support','Write — Right carriage bearing support'}|{p['id'] for p in meta if p['bank']=='Planar'}
def pose(p,qm,qe):
 T=np.eye(4);mo=p['motion']
 if mo=='carriage':T[0,3]=qm if p['bank']=='Memory' else -qe
 if mo=='bolt':T[2,3]=min(0,2*(qe-4.3))
 if mo=='bell-crank':T=trimesh.transformations.rotation_matrix(np.arcsin(qe/10),[0,1,0],[20,0,-52.8])
 return T
hits={};samples=[(qm,qe) for qm in [-4.3,4.3] for qe in np.linspace(-4.6,4.6,93)]+[(qm,-4.3) for qm in np.linspace(-4.6,4.6,93)]
for qm,qe in samples:
 ts={p['id']:pose(p,qm,float(qe)) for p in meta}
 for n,T in ts.items():cm.set_transform(n,T)
 _,pairs=cm.in_collision_internal(return_names=True)
 for a,b in pairs:
  if not ({a,b}&changed):continue
  if ('Short lever' in a or 'Short lever' in b):continue
  key=tuple(sorted([a,b]))
  if key in hits:continue
  ix=sol[a].transform(ts[a][:3,:])^sol[b].transform(ts[b][:3,:]);vol=max(0,float(ix.volume()))
  if vol>.005:
   vv=np.asarray(ix.to_mesh64().vert_properties)[:,:3];hits[key]=dict(a=a,b=b,volume=vol,qm=float(qm),qe=float(qe),bounds=[vv.min(0).tolist(),vv.max(0).tolist()])
rep=dict(poses=len(samples),interferences=list(hits.values()),limitations=['Source lever pose excluded from this independent clearance sweep.','Hardware, elastic band route and strength not qualified.'])
(O/'Checks.json').write_text(json.dumps(rep,indent=2));print(json.dumps(rep,indent=2))
