"""Projected native 16T profiles: find compatible initial phases, not tooth strength."""
from pathlib import Path
import json,math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
R=Path(__file__).resolve().parents[1];O=R/'Assembly development'
ps=json.loads((O/'parts.json').read_text());vs=np.load(O/'geometry.npz')['vertices']
def profile(name):
 p=next(p for p in ps if p['id']==name);v=vs[p['offset']//3:p['offset']//3+p['vertices']].reshape(-1,3,3)[:,:,1:].copy();c=np.array(p['centre'])[1:];v-=c
 a=v[:,1]-v[:,0];b=v[:,2]-v[:,0];ok=abs(a[:,0]*b[:,1]-a[:,1]*b[:,0])>1e-7
 return unary_union([Polygon(t) for t in v[ok]]),c
p,_=profile('Q return gear 0');records=[]
for prefix,count in [('Q return gear ',5),('D route gear ',7)]:
 phase=0.
 for i in range(count-1):
  a,ca=profile(prefix+str(i));b,cb=profile(prefix+str(i+1));dy,dz=cb-ca
  def overlap(theta,offset):
   aa=affinity.rotate(a,theta+phase,origin=(0,0));bb=affinity.translate(affinity.rotate(b,-theta+offset,origin=(0,0)),dy,dz)
   return aa.intersection(bb).area
  coarse=[(max(overlap(t,o) for t in np.linspace(0,22.5,16)),o) for o in np.arange(0,22.5,.25)]
  area,off=min(coarse)
  fine=[(max(overlap(t,o) for t in np.linspace(0,22.5,91)),o) for o in np.arange(0,22.5,.125)]
  valid=[o for area,o in fine if area<1e-8]
  if valid:
   # Centre the largest contiguous collision-free phase interval.
   groups=[]
   for o in valid:
    if not groups or o-groups[-1][-1]>.13:groups.append([])
    groups[-1].append(o)
   if len(groups)>1 and groups[0][0]==0 and groups[-1][-1]>=22.375:groups=[groups[-1]+[o+22.5 for o in groups[0]]]+groups[1:-1]
   group=max(groups,key=len);off=((group[0]+group[-1])/2)%22.5;area=max(overlap(t,off) for t in np.linspace(0,22.5,181))
  else:area,off=min(fine)
  records.append(dict(a=prefix+str(i),b=prefix+str(i+1),phase_a_deg=phase,phase_b_deg=off,max_overlap_mm2=area,centre_distance=float(np.linalg.norm(cb-ca))))
  phase=off
print(json.dumps(records,indent=2));(R/'Route tooth phase study.json').write_text(json.dumps(dict(scope='Projected gear envelopes over one tooth pitch; proposed phases require keyed-axle and dog-phase consistency checks',pairs=records),indent=2))
