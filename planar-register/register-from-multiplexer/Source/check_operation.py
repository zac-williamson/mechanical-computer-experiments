"""Sampled whole-assembly kinematic audit, including every input-pair transition.
Prescribed motion is not a force-driven simulation or a continuous collision proof.
"""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
s=(R/'Source/full_contact_inventory.py').read_text();exec(s[:s.index('records={}')])
P={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in ps}
S={n:m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True))) for n,t in P.items()}
paths=[];samples=[]
for d0 in (0,1):
 for w0 in (0,1):
  for bit in ((d0,) if w0 else (0,1)):
   for d1 in (0,1):
    for w1 in (0,1):
     qm=3.75-7.5*bit; qe=7.5*w0-3.75; row=[]
     # Disconnect/release first; then move the stored carriage if WRITE is active.
     for e in np.linspace(qe,7.5*w1-3.75,41):row.append((qm,float(e)))
     if w1:
      for q in np.linspace(qm,3.75-7.5*d1,76):row.append((float(q),3.75))
     paths.append(dict(initial=[d0,w0,bit],target=[d1,w1],samples=len(row)));samples+=row
# Interruptions, reversals and resumption at three intermediate data positions.
interruptions=[]
for q in (-1.875,0,1.875):
 for d in (0,1):
  row=[(q,float(e)) for e in np.linspace(3.75,-3.75,41)]
  row += [(q,float(e)) for e in np.linspace(-3.75,3.75,41)]
  row += [(float(x),3.75) for x in np.linspace(q,3.75-7.5*d,76)]
  samples+=row;interruptions.append(dict(interrupted_q=q,resume_D=d,hold_output_valid=False))
# Independent overtravel envelope also covers overlap of two actuator motions.
samples += [(float(q),float(e)) for q in np.linspace(-4.6,4.6,47) for e in np.linspace(-4.6,4.6,47)]
samples=list(dict.fromkeys((round(q,6),round(e,6)) for q,e in samples))
cache={};hits={};native_pairs={};counts=0
for i,(qm,qe) in enumerate(samples):
 T={p['id']:pose(p,qm,qe) for p in meta}
 for n,t in T.items():cm.set_transform(n,t)
 _,pairs=cm.in_collision_internal(return_names=True)
 for a,b in pairs:
  a,b=sorted((a,b));relative=np.linalg.inv(T[a])@T[b]
  key=(a,b,tuple(np.round(relative[:3,:].ravel(),5)))
  if a in P and b in P:
   if key not in cache:cache[key]=max(0,float((S[a]^S[b].transform(relative[:3,:])).volume()))
   vol=cache[key]
   if vol>.005 and vol>hits.get((a,b),{}).get('volume_mm3',0):hits[a,b]=dict(pair=[a,b],volume_mm3=vol,sample=[qm,qe])
  else:
   native_pairs.setdefault((a,b),dict(pair=[a,b],samples=[]))
   # Preserve all distinct relative poses for a separate native surface audit.
   if key not in cache:
    native_pairs[a,b]['samples'].append([qm,qe]);cache[key]=None
 if i%500==0:print('Checked',i,'/',len(samples),'printed interference pairs',len(hits),flush=True)
report=dict(parts=len(meta),printed_parts=len(ps),hardware_parts=len(hs),unique_poses=len(samples),settled_state_transition_paths=paths,input_pairs_covered=sorted({tuple(p['initial'][:2]+p['target']) for p in paths}),interrupted_write_paths=interruptions,printed_interferences=list(hits.values()),native_contact_candidates=list(native_pairs.values()),limits=['Finite pose sampling; no continuous collision certification.','Native LEGO surfaces include open meshes; their candidate contacts require separate narrow-phase classification.','Actuator rotation follows the nearest source trace position; independent running gear phase is not swept here.','Prescribed kinematics, not load/friction simulation. Interrupted writes can retain an invalid intermediate output.'])
(O/'Operation checks.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['native_contact_candidates','settled_state_transition_paths']},indent=2))
