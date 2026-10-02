"""Native external gear meshes, shaft-consistent initial phases and unit ratios.

Does not model internal dog teeth or guarantee loaded clutch engagement.
"""
from pathlib import Path
import json,hashlib,math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
R=Path(__file__).resolve().parents[1]/'Compact layout';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
def group(n):
 if 'POWER 16T' in n:return 'POWER internal'
 if 'reversing idler' in n:return n.split()[0]+' idler'
 if n=='POWER header gear':return 'POWER input'
 if n=='D header input 8T':return 'D input'
 if n in ['D header output 8T','WRITE D input gear']:return 'D inverted'
 if n=='Q takeoff gear':return 'Q'
 if n.startswith('Q feedback front gear'):return 'Q inverted'
 if n=='Selected data route 16T':return 'Selected data'
 if n=='master_gate B-input':return 'Selected data inverted'
 if n=='slave_gate B-input':return 'Master output'
 if n.startswith('CLK header gear'):return 'CLK input' if n.endswith('0') else 'CLK worm'
 return n # Each clutch gear spins independently of its through axle.
items=[]
for p in parts:
 n=p['id'];num=p.get('lego_part');N={'94925':16,'3648':24,'10928':8}.get(num)
 if n.endswith((' L072',' L102',' B-input')):N=16
 if N is None:continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']];centre=(a.min(0)+a.max(0))/2;rad=np.linalg.norm(a[:,1:]-centre[1:],axis=1)
 face=a[rad>N/2+.15,0];tri=a.reshape(-1,3,3)[:,:,1:]-centre[1:];uv=tri[:,1]-tri[:,0];vv=tri[:,2]-tri[:,0];area=abs(uv[:,0]*vv[:,1]-uv[:,1]*vv[:,0]);profile=unary_union([Polygon(t) for t in tri[area>1e-8]])
 items.append(dict(name=n,N=N,centre=centre,face=[face.min(),face.max()],profile=profile,group=group(n)))
edges=[]
for i,a in enumerate(items):
 for b in items[i+1:]:
  distance=np.linalg.norm(a['centre'][1:]-b['centre'][1:])
  if abs(distance-(a['N']+b['N'])/2)>.05:continue
  if min(a['face'][1],b['face'][1])-max(a['face'][0],b['face'][0])<.1:continue
  edges.append((a,b))
phase={};records=[];remaining=edges[:]
while remaining:
 usable=next(((i,a,b) for i,(a,b) in enumerate(remaining) if a['group'] in phase or b['group'] in phase),None)
 if usable is None:
  a,b=remaining[0];phase[a['group']]=0.;usable=(0,a,b)
 i,a,b=usable;remaining.pop(i)
 if a['group'] not in phase:a,b=b,a
 pa=phase[a['group']];delta=b['centre'][1:]-a['centre'][1:];period=360/b['N'];theta=np.linspace(0,360/a['N'],49)
 rotated=[affinity.rotate(a['profile'],float(t+pa),origin=(0,0)) for t in theta]
 def maximum(pb):
  return max(aa.intersection(affinity.translate(affinity.rotate(b['profile'],float(-t*a['N']/b['N']+pb),origin=(0,0)),*delta)).area for t,aa in zip(theta,rotated))
 if b['group'] in phase:pb=phase[b['group']];overlap=maximum(pb)
 else:
  candidates=[(maximum(float(pb)),float(pb)) for pb in np.arange(0,period,.25)];clear=[pb for area,pb in candidates if area<1e-8]
  if clear:
   runs=[]
   for pb in clear:
    if not runs or pb-runs[-1][-1]>.26:runs.append([])
    runs[-1].append(pb)
   run=max(runs,key=len);pb=(run[0]+run[-1])/2;overlap=maximum(pb)
  else:overlap,pb=min(candidates)
  phase[b['group']]=pb
 records.append(dict(a=a['name'],b=b['name'],shaft_a=a['group'],shaft_b=b['group'],centre_distance_mm=float(np.linalg.norm(delta)),ratio=-a['N']/b['N'],phase_a_deg=pa,phase_b_deg=pb,max_projected_overlap_mm2=overlap))
result=dict(scope=__doc__,geometry_sha256=digest,shaft_phases_deg=phase,meshes=records,all_unit_magnitude=all(abs(e['ratio'])==1 for e in records),external_profile_pass=all(e['max_projected_overlap_mm2']<1e-8 for e in records),mechanically_qualified=False)
(R/'External gear phase checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
