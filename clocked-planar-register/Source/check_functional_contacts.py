"""Check cassette-specific geometry against production mechanisms and docked scene."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
from wall_pose import vertices,transform,joint
from wall_flat_frame import native_envelope
R=Path(__file__).resolve().parents[1];O=R/'Functional register'
P=json.load(open(O/'parts.json'));V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);D=json.load(open(O/'Architecture.json'))
NEW=set(D['print_orientations']);CASES=json.load(open(R/'Compact layout/Compact contact-resolved operation.json'))['cases']
frames=[c['frames'][i] for c in CASES for i in np.linspace(0,len(c['frames'])-1,9,dtype=int)]
def a(p):return V[p['offset']//3:p['offset']//3+p['vertices']]
def solid(v):
 t=trimesh.Trimesh(v,np.arange(len(v)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
scene=[]
for p in P:
 if p['kind']=='elastic':continue
 v=a(p)
 s=native_envelope(p,v,joint,frames[0]) if p['kind']=='native' else solid(v)
 scene.append((p,v,s))
# Cache transforms once. Previously every pair recomputed and inverted the
# same 1,008 transforms, wasting CPU without adding any geometric coverage.
# About 36 MiB of matrices; still one worker under the existing RSS guard.
TF=[np.array([transform(p,f) for f in frames]) for p,v,s in scene]
INV={i:np.linalg.inv(TF[i]) for i,(p,v,s) in enumerate(scene) if p['id'] in NEW}
hits=[];tested=0
for i,(p,v,s) in enumerate(scene):
 if p['id'] not in NEW:continue
 print('CONTACT',p['id'],flush=True)
 for j,(q,w,t) in enumerate(scene):
  if i==j or q['id'] in NEW and j<i:continue
  # Interference grip of friction-pin ribs is intentional. Socket seating is
  # audited separately; this contact check still includes axles and gears.
  if q['kind']=='native' and (q.get('lego_part') in ['2780','6558'] or ' pin ' in q['id']):continue
  relative=INV[i]@TF[j]
  _,indices=np.unique(np.round(relative.reshape(len(frames),16),4),axis=0,return_index=True)
  for fi in sorted(indices):
   f=frames[fi];rel=relative[fi]
   bb=np.array(s.bounding_box()).reshape(2,3);ww=trimesh.transform_points(w,rel)
   if np.any(np.minimum(bb[1],ww.max(0))-np.maximum(bb[0],ww.min(0))<=.001):continue
   tested+=1;vol=(s^t.transform(rel[:3])).volume()
   if vol>.025:
    hit=dict(a=p['id'],b=q['id'],volume_mm3=float(vol),rail=f['rail']);hits.append(hit);print('HIT',hit,flush=True);break
(O/'Candidate contact checks.json').write_text(json.dumps(dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),checks=tested,sampled_frames=len(frames),collisions=hits,pass_check=not hits,scope='New printed parts against sampled inherited motion; native rotation envelopes. No slicer or physical validation.'),indent=2))
print('TOTAL',tested,'HITS',len(hits),flush=True)
