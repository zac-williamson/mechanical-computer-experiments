from pathlib import Path
import json,itertools,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Linkage development';meta=json.loads((O/'printed-parts.json').read_text());parts={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in meta}
changed={'Memory — Fixed lower latch guide','Memory — Lock bolt','Write — Right carriage bearing support','Common two-core baseboard','Lock crosshead','Lock rocker','Write cam','Rear linkage frame','Upper cam rail'}
def pose(p,qm,qe,bolt,command=None):
 T=np.eye(4);mo=p['motion']
 if mo=='carriage':T[0,3]=qm if p['bank']=='Memory' else -qe
 if mo=='write-cam':T[0,3]=-qe
 if mo=='bolt':T[2,3]=bolt
 if mo=='rocker':T=trimesh.transformations.rotation_matrix(np.arcsin((5.8+(command if command is not None else (-15+20*np.clip((qe-.9)/1.5,0,1))))/50),[0,1,0],[50,0,2.7])
 return T
solids={n:m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64))) for n,t in parts.items()}
cm=trimesh.collision.CollisionManager()
for n,t in parts.items():cm.add_object(n,t)
hits={};samples=[]
for qe in np.linspace(-4.6,4.6,93):
 cmd=-15+20*np.clip((qe-.9)/1.5,0,1)
 for qm in [-4.3,4.3]:samples.append((qm,float(qe),min(0,float(cmd))))
for qm in np.linspace(-4.6,4.6,93):samples.append((float(qm),-4.3,-15))
samples=[(*s,None) for s in samples]
seq=json.loads((R/'Investigation/Sequencing study.json').read_text())
for row in seq['samples']:
 for cmd in [row['command_min_mm'],row['command_max_mm']]:
  if cmd is not None:
   for qm in [-4.3,4.3]:samples.append((qm,row['q'],min(0,cmd),cmd))
# Mid-stroke keeper blocks insertion at first tip contact; band must absorb remaining drive.
for qm in np.linspace(-3.0,3.0,31):samples.append((float(qm),4.3,-9.2,None))
for i,(qm,qe,bolt,cmd) in enumerate(samples):
 transforms={p['id']:pose(p,qm,qe,bolt,cmd) for p in meta}
 for n,T in transforms.items():cm.set_transform(n,T)
 _,pairs=cm.in_collision_internal(return_names=True)
 for na,nb in pairs:
  if not ({na,nb}&changed):continue
  if {na,nb}=={'Write — Right carriage bearing support','Write — Short lever'}:continue # inherited dynamic lever requires source contact trace
  key=tuple(sorted((na,nb)))
  if key in hits:continue
  a=solids[na].transform(transforms[na][:3,:]);b=solids[nb].transform(transforms[nb][:3,:]);vol=max(0,float((a^b).volume()))
  if vol>.005:hits[key]=dict(a=na,b=nb,volume_mm3=vol,memory_q=qm,write_q=qe,bolt=bolt)
report=dict(status='DEVELOPMENT — not released for printing',sample_count=len(samples),printed_interferences=list(hits.values()),limits=['Nominal motion plus sampled cam-clearance extremes and blocked mid-stroke insertion. Inherited write lever/support pair excluded because its contact-trace pose is not represented by this independent sweep.','Elastic band and hardware interference/strength not yet qualified.'])
(O/'Linkage checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
