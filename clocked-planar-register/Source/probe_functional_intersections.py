from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from wall_pose import transform
R=Path(__file__).resolve().parents[1];O=R/'Functional register'
P={p['id']:p for p in json.load(open(O/'parts.json'))};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
FS=[f for c in json.load(open(R/'Compact layout/Compact contact-resolved operation.json'))['cases'] for f in c['frames']]
def solid(p):
 a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
for hit in json.load(open(O/'Candidate contact checks.json'))['collisions']:
 p,q=P[hit['a']],P[hit['b']];s,t=solid(p),solid(q)
 best=None
 for f in FS:
  if abs(f['rail']-hit['rail'])>1e-7:continue
  rel=np.linalg.inv(transform(p,f))@transform(q,f);c=s^t.transform(rel[:3]);vol=c.volume()
  if best is None or vol>best[0]:best=(vol,c.bounding_box(),f)
  if abs(vol-hit['volume_mm3'])<.0001:break
 print(hit['a'],'VS',hit['b'],best[:2],flush=True)
