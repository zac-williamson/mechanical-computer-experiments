from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());hm=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];cp=trimesh.collision.CollisionManager();ch=trimesh.collision.CollisionManager()
for p in meta:cp.add_object(p['id'],trimesh.load(O/(p['id']+'.stl')))
for h in hm:
 a=v[h['offset']//3:h['offset']//3+h['vertices']];ch.add_object(h['id'],trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False))
def pose(p,qm,qe):
 T=np.eye(4);mo=p['motion'];q=qm if p['bank']=='Memory' else qe
 if mo in ['carriage','worm']:T[0,3]=q if p['bank']=='Memory' else -q
 if mo=='clutch-ring':T[0,3]=(1 if p['bank']=='Memory' else -1)*np.sign(q)*max(abs(q)-.4,0)
 if mo=='bolt':T[2,3]=min(0,2*(qe-4.3))
 if mo=='bell-crank':T=trimesh.transformations.rotation_matrix(np.arcsin(qe/10),[0,1,0],[20,0,-52.8])
 return T
hits={}
for qe in np.linspace(-4.6,4.6,31):
 for qm in [-4.3,4.3]:
  for p in meta:cp.set_transform(p['id'],pose(p,qm,qe))
  for h in hm:ch.set_transform(h['id'],pose(h,qm,qe))
  _,pairs=cp.in_collision_other(ch,return_names=True)
  for p,h in pairs:
   if not h.startswith('Planar'):continue
   if h=='Planar roller 0' and p=='Lock bolt':continue # intended withdrawal contact; native radial tessellation differs by <0.0001 mm
   if h.startswith('Planar guide joint') and p in ['Common planar base','Upper lock guide']:continue
   if h.startswith('Planar pivot') and p=='Common planar base':continue
   hits.setdefault((p,h),dict(printed=p,hardware=h,qm=qm,qe=float(qe)))
report=dict(poses=62,new_hardware_contacts=list(hits.values()),limits=['New hardware only; not an all-phase, all-part hardware collision certification.','5L axle end geometry approximated from native cross section.','No force or band-path qualification.']);(O/'Hardware checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
