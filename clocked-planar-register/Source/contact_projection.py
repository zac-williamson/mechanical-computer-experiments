"""Experimental position-level correction of table-quantised actuator contacts.

Preserves worm lead exactly while resolving reaction/lever and roof/lever
penetration. A feasible corrected pose is not a force or stability proof.
"""
from pathlib import Path
from functools import lru_cache
import json,math
import numpy as np,trimesh,manifold3d as m
from shapely import affinity
from scipy.optimize import minimize, differential_evolution
from actuator import project,PIVOT,LEAD,TEETH
B=Path(__file__).resolve().parents[2]/'planar-register/work/register-mux-reference/multiplexer'
@lru_cache(maxsize=1)
def geometry():
 h=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];p=next(p for p in h if p['id']=='U022');a=v[p['offset']//3:p['offset']//3+p['vertices']]
 gear=project(trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False));lever=project(trimesh.load(B/'Short lever.stl'))
 mesh=trimesh.load(B/'Carriage fork and roof.stl');s=m.Manifold(m.Mesh64(mesh.vertices.astype(float),mesh.faces.astype(np.uint64)));q=(s^m.Manifold.cube([100,8.38,100]).translate([-50,6.01,-30])).to_mesh64();roof=project(trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=False))
 return gear,lever,roof

def correct(q,g,b,limit=.5):
 gear,lever,roof=geometry();factor=LEAD*TEETH/360
 def polygons(x):
  dg,db=x;return affinity.rotate(gear,-g-dg,origin=(0,24)),affinity.rotate(lever,-b-db,origin=PIVOT),affinity.translate(roof,xoff=q-factor*dg)
 def areas(x):
  gg,ll,rr=polygons(x);return np.array([ll.intersection(gg).area,ll.intersection(rr).area])
 before=areas([0,0])
 if before.max()<1e-14:return dict(q=q,g=g,b=b,areas=before.tolist(),changed=False)
 def clearance(x):return 1e-7-np.sqrt(areas(x))
 candidates=[]
 for start in ([0.,0.],[.05,-.05],[-.05,.05],[.2,-.2],[-.2,.2]):
  result=minimize(lambda x:float(x@x),start,method='SLSQP',bounds=[(-limit,limit)]*2,constraints=[dict(type='ineq',fun=clearance)],options=dict(maxiter=120,ftol=1e-13,eps=1e-6))
  aa=areas(result.x)
  if aa.max()<1e-12:candidates.append((float(result.x@result.x),result.x,aa))
  if candidates:break
 if not candidates:
  # Flat zero-overlap regions make the local constrained solver unreliable
  # at a few tooth-tip poses. Search the bounded correction region globally;
  # still reject any residual overlap rather than relaxing the threshold.
  result=differential_evolution(lambda x:float(areas(x).sum())+1e-10*float(x@x),[(-limit,limit)]*2,seed=1,maxiter=120,popsize=10,tol=1e-9,polish=False)
  aa=areas(result.x)
  if aa.max()<1e-12:candidates.append((float(result.x@result.x),result.x,aa))
 if not candidates:return dict(q=q,g=g,b=b,areas=before.tolist(),failed=True)
 _,x,aa=min(candidates,key=lambda z:z[0]);dg,db=x
 return dict(q=float(q-factor*dg),g=float(g+dg),b=float(b+db),areas=aa.tolist(),changed=True,delta_g_deg=float(dg),delta_b_deg=float(db))
if __name__=='__main__':
 import time
 t=time.monotonic();print(correct(-3.755874,-29.619554207233925,15.251473711861227),time.monotonic()-t)
 R=Path(__file__).resolve().parents[1]/'Compact layout';r=json.loads((R/'Contact-solved printed motion.json').read_text());frames=[f for c in json.loads((R/'Compact angle-driven operation.json').read_text())['cases'] for f in c['frames']]
 for hit in r['intersections']:
  bank=hit['a'].split()[0];f=frames[hit['frame']][bank];print(bank,f,correct(f['q'],f['g'],f['b']),time.monotonic()-t,flush=True)
