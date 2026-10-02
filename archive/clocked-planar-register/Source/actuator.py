"""Rigid quasi-static actuator: worm lead plus native gear/printed contact constraints.
Angles degrees, distances mm. Not a strength/dynamic-friction solver.
"""
from pathlib import Path
import json,hashlib, numpy as np,trimesh,shapely,manifold3d as manifold
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
from bisect import bisect_left,bisect_right
_PACKED={}
def packed(table):
 key=id(table)
 if key not in _PACKED:
  masks=lambda a:[int.from_bytes(np.packbits(row,bitorder="little").tobytes(),"little") for row in a]
  bs=table["bs"].tolist()
  _PACKED[key]=(table,bs,masks(table["gc"]<=.00021),masks(table["rc"]<=.00025),min(range(len(bs)),key=lambda i:abs(bs[i])))
 return _PACKED[key]
B=Path(__file__).resolve().parents[2]/'work/register-mux-reference/multiplexer'
LEAD=np.pi; TEETH=8; PIVOT=(13.192323603988665,32.12844869819761)
def project(mesh):
 t=mesh.triangles[:,:,[0,2]];u=t[:,1]-t[:,0];v=t[:,2]-t[:,0];a=u[:,0]*v[:,1]-u[:,1]*v[:,0];return unary_union([Polygon(x) for x in t[np.abs(a)>1e-9]])
def tables():
 cache=Path(__file__).resolve().parents[2]/'planar-register/register-from-multiplexer/Planar register/Actuator contact tables clipped.npz'
 return dict(np.load(cache))
if __name__=='__main__':
 t=tables();print({k:v.shape for k,v in t.items()});f=json.loads((B/'Switching trace.json').read_text())['frames'];errs=[]
 for r in f:
  bi=np.argmin(abs(t['bs']-r['b']));gi=int(round(r['g']/.25))%len(t['gs']);qi=np.argmin(abs(t['qs']-r['q']));errs.append((t['gc'][gi,bi],t['rc'][qi,bi]))
 print('Reference max profile overlaps',np.max(errs,axis=0))

class Actuator:
 def __init__(self,side=1,table=None):
  self.t=table or tables();_,self.bs,self.G,self.R,self.zero=packed(self.t);self.q=3.749041 if side>0 else -3.755874;self.g=98.25 if side>0 else -69.25;self.b=-20.8 if side>0 else 24.3;self.w=720. if side>0 else -1480.;self.mode='overrun';self.stalled=False
 def allowed(self,q,g):
  t=self.t;gi=int(round(g/.25))%180;qi=int(np.clip(round((q+4.5)/.01),0,len(t['qs'])-1));return (t['gc'][gi]<=.00021)&(t['rc'][qi]<=.00025)
 def step(self,dw,locked=False):
  # Integrate shaft rotation; never prescribe a carriage position.
  remaining=dw;accepted=0.;self.stalled=False
  while abs(remaining)>1e-9:
   step=np.sign(remaining)*min(abs(remaining),2.);ng=self.g+step/TEETH
   gi=int(round(ng/.25))%180;qi=max(0,min(len(self.R)-1,round((self.q+4.5)/.01)))
   lo=bisect_left(self.bs,self.b-1.01);hi=bisect_right(self.bs,self.b+1.01)
   mask=self.G[gi]&self.R[qi]&((1<<hi)-(1<<lo))
   if mask:
    lower=mask&((1<<(self.zero+1))-1);upper=mask>>self.zero;candidates=[]
    if lower:candidates.append(lower.bit_length()-1)
    if upper:candidates.append(self.zero+(upper&-upper).bit_length()-1)
    bi=min(candidates,key=lambda i:(abs(self.bs[i]),i));self.g=ng;self.b=self.bs[bi];self.mode='reaction gear rotating / carriage stationary'
   else:
    nq=self.q+LEAD*step/360;bi=int(round((self.b+30)/.1));gi=int(round(self.g/.25))%180;qi=max(0,min(len(self.R)-1,round((nq+4.5)/.01)));okq=(self.G[gi]&self.R[qi])&(1<<bi)
    if locked or not okq or abs(nq)>4.45:self.stalled=True;self.mode='blocked: input rotation refused';break
    self.q=nq;self.mode='reaction gear restrained / carriage translating'
   self.w+=step;accepted+=step;remaining-=step
  return accepted
 def state(self):return dict(q=self.q,w=self.w,g=self.g,b=self.b,mode=self.mode,stalled=self.stalled)
