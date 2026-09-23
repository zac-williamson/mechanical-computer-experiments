from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];cp=trimesh.collision.CollisionManager();ch=trimesh.collision.CollisionManager()
for p in meta:cp.add_object(p['id'],trimesh.load(O/(p['id']+'.stl')))
selected=[]
for h in hs:
 if h['id'].startswith(('Memory — A-','Memory — B-','Write — B-')):
  a=v[h['offset']//3:h['offset']//3+h['vertices']];ch.add_object(h['id'],trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False));selected.append(h)
hits=set()
for q in np.linspace(-4.6,4.6,19):
 for p in meta:
  T=np.eye(4)
  if p['bank']=='Memory' and p['motion']=='carriage':T[0,3]=q
  cp.set_transform(p['id'],T)
 for deg in np.arange(0,360,5):
  for h in selected:
   n=h['id'];c=[0,10.2,0] if n.startswith('Write') else [0,10.2+np.sqrt(33.75),-10.5] if 'idler' in n else [0,10.2,-16];T=trimesh.transformations.rotation_matrix(np.radians(deg),[1,0,0],c);ch.set_transform(n,T)
  _,pairs=cp.in_collision_other(ch,return_names=True);hits.update(pairs)
report=dict(samples=1368,printed_contacts=[list(p) for p in sorted(hits)],note='Changed memory hardware vs printed structure; axle and bush bearing-face contacts require interpretation; gear topology checked separately.')
(O/'Memory hardware checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
