"""Conservative solid-cylinder screening of axle/pin crossings.

A reported crossing requires native mesh review; cross-shaped axles do not
fill their circular envelopes. Coaxial touching ends are retained as contacts.
"""
import json,hashlib
from pathlib import Path
import numpy as np,manifold3d as m
R=Path(__file__).resolve().parents[1]/'Wall register';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
p=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3);shafts=[]
nums={'2780','23948','44294','60485','4519','32062','32073','24316','3705','3706','3707','3708','3737','50450','50451'}
for part in p:
 if part.get('lego_part') not in nums and not any(tag in part['id'] for tag in ['stop-axle','Carriage support pin']):continue
 a=v[part['offset']//3:part['offset']//3+part['vertices']];lo=a.min(0);hi=a.max(0);axis=part.get('axis',int(np.argmax(hi-lo)));c=(lo+hi)/2
 s=m.Manifold.cylinder(hi[axis]-lo[axis],2.4,circular_segments=32)
 if axis==0:s=s.rotate([0,90,0])
 elif axis==1:s=s.rotate([-90,0,0])
 pos=c.copy();pos[axis]=lo[axis];s=s.translate(pos);shafts.append((part,s,np.array(s.bounding_box()).reshape(2,3)))
hits=[];end_gaps=[]
for i,(p,s,b) in enumerate(shafts):
 for q,t,bb in shafts[i+1:]:
  pa=p.get('axis',int(np.argmax(b[1]-b[0])));qa=q.get('axis',int(np.argmax(bb[1]-bb[0])))
  cross=[a for a in range(3) if a!=pa]
  if pa==qa and np.linalg.norm(((b[0]+b[1]-bb[0]-bb[1])/2)[cross])<.01:
   gap=max(b[0,pa]-bb[1,pa],bb[0,pa]-b[1,pa])
   if -.001<=gap<.5:end_gaps.append(dict(a=p['id'],b=q['id'],axial_gap_mm=float(gap)))
  if np.any(np.minimum(b[1],bb[1])-np.maximum(b[0],bb[0])<=0):continue
  vol=(s^t).volume()
  if vol>.01:hits.append(dict(a=p['id'],b=q['id'],envelope_volume_mm3=vol))
result=dict(scope=__doc__,geometry_sha256=digest,potential_crossings=hits,coaxial_end_gaps_below_0_5_mm=end_gaps,mechanically_qualified=False)
(R/'Axle crossing screening.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
