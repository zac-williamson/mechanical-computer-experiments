"""Projected axial-tip profiles for clutch face-engagement investigation.

Diagnostic only. Surface projection of an open LDraw mesh is not a certified
solid section; it is used to locate phase-dependent face blocking for review.
"""
from pathlib import Path
import json,hashlib
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
R=Path(__file__).resolve().parents[1]/'Compact layout';ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
def triangles(name):
 p=next(p for p in ps if p['id']==name);return v[p['offset']//3:p['offset']//3+p['vertices']].reshape(-1,3,3)
def clip(poly,x,side):
 out=[]
 for a,b in zip(poly,np.roll(poly,-1,axis=0)):
  ia=side*(a[0]-x)>=0;ib=side*(b[0]-x)>=0
  if ia:out.append(a)
  if ia!=ib:out.append(a+(b-a)*(x-a[0])/(b[0]-a[0]))
 return np.array(out)
def profile(tri,x,side):
 pp=[]
 for t in tri:
  p=clip(t,x,side)
  if len(p)<3:continue
  q=Polygon(p[:,1:]-[10.2,16])
  if q.area>1e-10:pp.append(q)
 return unary_union(pp)
a=triangles('master_gate L099');b=triangles('master_gate L102');tip=a[:,:,0].max();front=b[:,:,0].min();rows=[]
for depth in [.05,.5,1.,2.,2.5]:
 aa=profile(a,tip-depth,1);bb=profile(b,front+depth,-1)
 values=[float(affinity.rotate(aa,float(theta),origin=(0,0)).intersection(bb).area) for theta in np.arange(0,360,.5)]
 rows.append(dict(overlap_depth_mm=depth,ring_profile_area_mm2=aa.area,gear_profile_area_mm2=bb.area,clear_phases_deg=[i*.5 for i,a in enumerate(values) if a<1e-8],max_projected_overlap_mm2=max(values)))
 print(depth,'clear phases',sum(a<1e-8 for a in values),'of',len(values),'max overlap',max(values),flush=True)
report=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),nominal_first_axial_overlap_mm=float(front-tip),profiles=rows,mechanically_qualified=False)
(R/'Clutch face phase study.json').write_text(json.dumps(report,indent=2))
