from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());hm=json.loads((O/'hardware.json').read_text());vv=np.load(O/'hardware.npz')['vertices'];P={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in meta};sol={n:m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64))) for n,t in P.items()};cm=trimesh.collision.CollisionManager();ch=trimesh.collision.CollisionManager()
for n,t in P.items():cm.add_object(n,t)
for h in hm:
 a=vv[h['offset']//3:h['offset']//3+h['vertices']];ch.add_object(h['id'],trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False))
def pose(p,qm,qe,bolt=None):
 T=np.eye(4);mo=p['motion'];q=qm if p['bank']=='Memory' else qe
 if mo in ['carriage','worm']:T[0,3]=q
 if mo=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
 if mo=='bolt':T[2,3]=max(0,54+18*np.sin(np.arctan2(6.8,10)+np.arcsin((qe-6.8)/np.sqrt(146.24)))+8*np.cos(np.arctan2(6.8,10)+np.arcsin((qe-6.8)/np.sqrt(146.24)))+3.6-57) if bolt is None else bolt
 if mo=='release-crank':T=trimesh.transformations.rotation_matrix(np.arctan2(6.8,10)+np.arcsin((qe-6.8)/np.sqrt(146.24)),[0,1,0],[60,0,54])
 return T
changed={'Memory — Carriage fork and roof','Write — Carriage fork and roof','Write — Flat pin-mounted link'}|{p['id'] for p in meta if p['bank']=='Lock'}
hits={};hwhits={};samples=[(qm,float(qe),None) for qm in [-4.3,4.3] for qe in np.linspace(-4.6,4.6,93)]+[(float(qm),4.3,None) for qm in np.linspace(-4.6,4.6,93)]+[(float(qm),-4.3,5.4) for qm in np.linspace(-3,3,31)]
for qm,qe,bolt in samples:
 ts={p['id']:pose(p,qm,qe,bolt) for p in meta}
 for n,T in ts.items():cm.set_transform(n,T)
 for h in hm:ch.set_transform(h['id'],pose(h,qm,qe,bolt))
 _,pairs=cm.in_collision_internal(return_names=True)
 for a,b in pairs:
  if not ({a,b}&changed) or 'Short lever' in a or 'Short lever' in b:continue
  key=tuple(sorted([a,b]))
  if key in hits:continue
  ix=sol[a].transform(ts[a][:3,:])^sol[b].transform(ts[b][:3,:]);vol=max(0,float(ix.volume()))
  if vol>.005:
   v=np.asarray(ix.to_mesh64().vert_properties)[:,:3];hits[key]=dict(a=a,b=b,qm=qm,qe=qe,bolt=bolt,volume_mm3=vol,bounds=[v.min(0).tolist(),v.max(0).tolist()])
 _,pairs=cm.in_collision_other(ch,return_names=True)
 for p,h in pairs:
  if not (h.startswith('Lock ') or h.startswith('WRITE ')):continue
  if h.startswith('WRITE input retainer') and p=='Adjacent lock frame':continue # axial bearing face contact
  if h=='Lock rear pivot retainer' and p=='Adjacent lock frame':continue
  if h=='Lock pivot retainer' and p=='Adjacent lock frame':continue
  if h=='Lock pivot stop axle' and p=='Adjacent lock frame':continue
  if h=='Lock half bush 42 14.4' and p=='Adjacent lock bolt':continue # deliberate roller/head bearing contact
  hwhits.setdefault((p,h),dict(printed=p,hardware=h,qm=qm,qe=qe))
report=dict(poses=len(samples),printed_interferences=list(hits.values()),new_hardware_contacts=list(hwhits.values()),limits=['Source short-lever poses excluded from independent sweep; tooth-contact checks remain separate.','New hardware checked; complete rotating-hardware sweep not yet established.','Prescribed rigid motion, no force/friction/elasticity dynamics.'])
(O/'Adjacent checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
