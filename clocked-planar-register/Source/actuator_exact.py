"""Experimental adaptive rigid contact solver; not yet used by the release model.

Checks continuous-position silhouettes instead of rounding carriage position
into a contact table. Worm rotation is consumed by reaction rotation or lead-
constrained translation; the lever is held still during translation.
"""
from pathlib import Path
from functools import lru_cache
import math,json
import numpy as np,trimesh,shapely,manifold3d as m
from shapely import affinity
from actuator import project,PIVOT,LEAD,TEETH
B=Path(__file__).resolve().parents[2]/'planar-register/work/register-mux-reference/multiplexer'
@lru_cache(maxsize=1)
def tables():
 h=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];p=next(p for p in h if p['id']=='U022');a=v[p['offset']//3:p['offset']//3+p['vertices']]
 gear=project(trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False));lever=project(trimesh.load(B/'Short lever.stl'))
 mesh=trimesh.load(B/'Carriage fork and roof.stl');s=m.Manifold(m.Mesh64(mesh.vertices.astype(float),mesh.faces.astype(np.uint64)));q=(s^m.Manifold.cube([100,8.38,100]).translate([-50,6.01,-30])).to_mesh64();roof=project(trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=False))
 bs=np.arange(-30,30.0001,.005);ls=np.array([affinity.rotate(lever,-float(b),origin=PIVOT) for b in bs],dtype=object)
 return dict(gear=gear,roof=roof,lever=lever,bs=bs,ls=ls)
@lru_cache(maxsize=32768)
def allowed(q,g,bi):
 t=tables();lo=max(0,bi-201);hi=min(len(t['bs']),bi+202);ls=t['ls'][lo:hi]
 roof=affinity.translate(t['roof'],xoff=q);gear=affinity.rotate(t['gear'],-g,origin=(0,24))
 ar=shapely.area(shapely.intersection(ls,roof));ag=shapely.area(shapely.intersection(ls,gear));mask=(ar<1e-10)&(ag<1e-10)
 if mask.any():return t['bs'][np.flatnonzero(mask)+lo]
 # A feasible contact interval can be narrower than the angular grid.
 # Refine its continuous minimum instead of declaring an artificial jam.
 from scipy.optimize import minimize_scalar
 i=int(np.argmin(ar+ag));middle=float(t['bs'][lo+i])
 def area(beta):
  lever=affinity.rotate(t['lever'],-beta,origin=PIVOT)
  return lever.intersection(roof).area+lever.intersection(gear).area
 result=minimize_scalar(area,bounds=(max(-30,middle-.01),min(30,middle+.01)),method='bounded',options={'xatol':1e-11})
 return np.array([result.x]) if result.fun<1e-10 else np.array([])
class Actuator:
 def __init__(self,side=1,table=None):
  self.t=tables();self.q=3.749041 if side>0 else -3.755874;self.g=98.25 if side>0 else -69.25;self.b=-20.8 if side>0 else 24.3;self.w=720. if side>0 else -1480.;self.mode='overrun';self.stalled=False
  ids=self.choices(self.q,self.g)
  if len(ids):self.b=float(ids[np.argmin(abs(ids-self.b))])
 def choices(self,q,g):return allowed(round(q,10),round(g%45,10),int(round((self.b+30)/.005)))
 def step(self,dw,locked=False):
  remaining=dw;accepted=0.;self.stalled=False
  while abs(remaining)>1e-8:
   step=np.sign(remaining)*min(abs(remaining),2.)
   while True:
    ng=self.g+step/TEETH;ids=self.choices(self.q,ng)
    if len(ids):
     self.g=ng;self.b=float(ids[np.argmin(abs(ids))]);self.mode='reaction gear rotating / carriage stationary';break
    nq=self.q+LEAD*step/360;ids=self.choices(nq,self.g);bi=int(round((self.b+30)/.005))
    if not locked and abs(nq)<4.45 and len(ids):
     # At entry/exit from the stop, gear rotation and translation can
     # share a step. Find the least translation admitting a clear pose,
     # while preserving the exact worm lead relation.
     low,high=0.,1.;chosen=ids
     for _ in range(9):
      alpha=(low+high)/2;qq=self.q+LEAD*step*alpha/360;gg=self.g+step*(1-alpha)/TEETH;possible=self.choices(qq,gg)
      if len(possible):high=alpha;chosen=possible
      else:low=alpha
     self.q+=LEAD*step*high/360;self.g+=step*(1-high)/TEETH
     self.b=float(chosen[np.argmin(abs(chosen-self.b))])
     self.mode='coupled stop contact / carriage translating';break
    step*=.5
    if abs(step)<1e-5:
     self.stalled=True;self.mode='blocked: no nonpenetrating contact continuation';return accepted
   self.w+=step;accepted+=step;remaining-=step
  return accepted
 def state(self):return dict(q=self.q,w=self.w,g=self.g,b=self.b,mode=self.mode,stalled=self.stalled)
if __name__=='__main__':
 import time
 start=time.monotonic()
 for side in [-1,1]:
  a=Actuator(side)
  for i in range(1800):
   a.step(-side*2)
   if a.stalled:break
  print(side,i,a.state(),time.monotonic()-start,flush=True)
