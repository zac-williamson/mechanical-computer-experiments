"""Rigid quasi-static actuator: worm lead plus native gear/printed contact constraints.
Angles degrees, distances mm. Not a strength/dynamic-friction solver.
"""
from pathlib import Path
import json,hashlib, numpy as np,trimesh,shapely,manifold3d as manifold
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
B=Path(__file__).resolve().parents[2]/'work/register-mux-reference/multiplexer'
LEAD=np.pi; TEETH=8; PIVOT=(13.192323603988665,32.12844869819761)
def project(mesh):
 t=mesh.triangles[:,:,[0,2]];u=t[:,1]-t[:,0];v=t[:,2]-t[:,0];a=u[:,0]*v[:,1]-u[:,1]*v[:,0];return unary_union([Polygon(x) for x in t[np.abs(a)>1e-9]])
def tables():
 cache=B.parents[1]/'integrated-cam-development/Actuator contact tables clipped.npz'
 fingerprint=hashlib.sha256(b''.join((B/n).read_bytes() for n in ['Short lever.stl','Carriage fork and roof.stl','hardware.npz'])).hexdigest()
 if cache.exists():
  saved=dict(np.load(cache))
  if str(saved.get('fingerprint',''))==fingerprint:return saved
 h=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];p=next(p for p in h if p['id']=='U022');a=v[p['offset']//3:p['offset']//3+p['vertices']];gear=project(trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False));lever=project(trimesh.load(B/'Short lever.stl'));roofmesh=trimesh.load(B/'Carriage fork and roof.stl');solid=manifold.Manifold(manifold.Mesh64(np.array(roofmesh.vertices,copy=True),np.array(roofmesh.faces,dtype=np.uint64,copy=True)));slab=manifold.Manifold.cube([100,8.38,100]).translate([-50,6.01,-30]);cut=(solid^slab).to_mesh64();roof=project(trimesh.Trimesh(np.array(cut.vert_properties)[:,:3],np.array(cut.tri_verts),process=False))
 bs=np.arange(-30,30.0001,.1);gs=np.arange(0,45,.25);qs=np.arange(-4.5,4.5001,.01)
 ls=np.array([affinity.rotate(lever,-float(b),origin=PIVOT) for b in bs],dtype=object)
 gc=np.array([shapely.area(shapely.intersection(ls,affinity.rotate(gear,-float(g),origin=(0,24)))) for g in gs])
 rc=np.array([shapely.area(shapely.intersection(ls,affinity.translate(roof,xoff=float(q)))) for q in qs])
 np.savez_compressed(cache,bs=bs,gs=gs,qs=qs,gc=gc,rc=rc,fingerprint=fingerprint);return dict(np.load(cache))
if __name__=='__main__':
 t=tables();print({k:v.shape for k,v in t.items()});f=json.loads((B/'Switching trace.json').read_text())['frames'];errs=[]
 for r in f:
  bi=np.argmin(abs(t['bs']-r['b']));gi=int(round(r['g']/.25))%len(t['gs']);qi=np.argmin(abs(t['qs']-r['q']));errs.append((t['gc'][gi,bi],t['rc'][qi,bi]))
 print('Reference max profile overlaps',np.max(errs,axis=0))

class Actuator:
 def __init__(self,side=1,table=None):
  self.t=table or tables();self.q=3.749041 if side>0 else -3.755874;self.g=98.25 if side>0 else -69.25;self.b=-20.8 if side>0 else 24.3;self.w=720. if side>0 else -1480.;self.mode='overrun';self.stalled=False
 def allowed(self,q,g):
  t=self.t;gi=int(round(g/.25))%180;qi=int(np.clip(round((q+4.5)/.01),0,len(t['qs'])-1));return (t['gc'][gi]<=.00021)&(t['rc'][qi]<=.00025)
 def step(self,dw,locked=False):
  # Integrate shaft rotation; never prescribe a carriage position.
  remaining=dw;accepted=0.;self.stalled=False
  while abs(remaining)>1e-9:
   step=np.sign(remaining)*min(abs(remaining),2.);ng=self.g+step/TEETH;ok=self.allowed(self.q,ng);ids=np.flatnonzero(ok & (abs(self.t['bs']-self.b)<=1.01))
   if len(ids):
    bi=ids[np.argmin(abs(self.t['bs'][ids]))];self.g=ng;self.b=float(self.t['bs'][bi]);self.mode='reaction gear rotating / carriage stationary'
   else:
    nq=self.q+LEAD*step/360;bi=int(round((self.b+30)/.1));okq=self.allowed(nq,self.g)[bi]
    if locked or not okq or abs(nq)>4.45:self.stalled=True;self.mode='blocked: input rotation refused';break
    self.q=nq;self.mode='reaction gear restrained / carriage translating'
   self.w+=step;accepted+=step;remaining-=step
  return accepted
 def state(self):return dict(q=self.q,w=self.w,g=self.g,b=self.b,mode=self.mode,stalled=self.stalled)
