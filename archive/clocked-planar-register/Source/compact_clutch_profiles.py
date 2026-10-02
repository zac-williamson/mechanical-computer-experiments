"""Conservative angular free windows from the native clutch tip geometry.

Axial-tip surface projection supplies a non-interpenetration screen. It is not
an axial force/chamfer model. Four-fold symmetry is checked conservatively.
"""
from pathlib import Path
import json,hashlib
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
R=Path(__file__).resolve().parents[1]/'Compact layout'
def clip(poly,x,side):
 out=[]
 for a,b in zip(poly,np.roll(poly,-1,axis=0)):
  ia=side*(a[0]-x)>=0;ib=side*(b[0]-x)>=0
  if ia:out.append(a)
  if ia!=ib:out.append(a+(b-a)*(x-a[0])/(b[0]-a[0]))
 return np.array(out)
def profile(tri,x,side,centre):
 out=[]
 for t in tri:
  p=clip(t,x,side)
  if len(p)<3:continue
  q=Polygon(p[:,1:]-centre)
  if q.area>1e-10:out.append(q)
 return unary_union(out)
def generate():
 ps={p['id']:p for p in json.loads((R/'parts.json').read_text())};v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3);rows={}
 def triangles(n):
  p=ps[n];return v[p['offset']//3:p['offset']//3+p['vertices']].reshape(-1,3,3)
 for bank in ['master','slave','write','master_gate','slave_gate']:
  for gear in (['L072','L102'] if '_gate' not in bank else ['L102']):
   key=bank+' '+gear;a=triangles(bank+' L099');b=triangles(key);centre=(a.reshape(-1,3).min(0)+a.reshape(-1,3).max(0))[1:]/2;side=1 if b[:,:,0].mean()>a[:,:,0].mean() else -1;depth=2.
   tip=a[:,:,0].max() if side==1 else a[:,:,0].min();front=b[:,:,0].min() if side==1 else b[:,:,0].max()
   aa=profile(a,tip-side*depth,side,centre);bb=profile(b,front+side*depth,-side,centre)
   def overlap(theta):return max(float(affinity.rotate(aa,theta+k*90,origin=(0,0)).intersection(bb).area) for k in range(4))
   def clear(theta):return overlap(theta)<1e-10
   grid=np.arange(0,90,.25);ok=[clear(float(q)) for q in grid];begins=[i for i in range(len(grid)) if ok[i] and not ok[i-1]]
   if len(begins)!=1:raise ValueError((key,'non-single free window',begins))
   j=begins[0];lo=float(grid[j]);hi=lo
   while clear(hi+.25):hi+=.25
   bad,good=lo-.25,lo
   for _ in range(32):
    mid=(bad+good)/2
    if clear(mid):good=mid
    else:bad=mid
   lower=good+1e-6;good,bad=hi,hi+.25
   for _ in range(32):
    mid=(bad+good)/2
    if clear(mid):good=mid
    else:bad=mid
   upper=good-1e-6
   rows[key]=dict(side=side,period_deg=90.,lower_deg=lower,upper_deg=upper,free_play_deg=upper-lower,nominal_axial_contact_mm=float(side*(front-tip)),profile_depth_mm=depth,native_geometry_sha256=hashlib.sha256(a.tobytes()+b.tobytes()).hexdigest())
   print(key,rows[key],flush=True)
 out=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),interfaces=rows,mechanically_qualified=False);(R/'Clutch angular windows.json').write_text(json.dumps(out,indent=2));return out
if __name__=='__main__':generate()
