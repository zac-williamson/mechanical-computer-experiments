"""Phase complete keyed shaft assemblies; phase loose clutch gears independently.
Native tooth silhouettes are a diagnostic, not a zero-volume contact certificate.
"""
from pathlib import Path
import numpy as np,trimesh,json,shapely
from shapely import affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];A={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in hs}
def proj(n):
 t=A[n].reshape(-1,3,3)[:,:,[1,2]];u=t[:,1]-t[:,0];v=t[:,2]-t[:,0];return unary_union([Polygon(x) for x in t[np.abs(u[:,0]*v[:,1]-u[:,1]*v[:,0])>1e-9]])
report=[]
for a,b,ca,cb,ra,rb,pitch,group in [
 ('Memory — A-input','Memory — A-idler',(10.2,-16),(10.2+np.sqrt(33.75),-10.5),1,-1,45,[p['id'] for p in hs if 'A-idler' in p['id']]),
 ('Memory — A-idler','Memory — L102',(10.2+np.sqrt(33.75),-10.5),(10.2,0),-1,.5,22.5,['Memory — L102']),
 ('Memory — B-input','Memory — L072',(10.2,-16),(10.2,0),1,-1,22.5,['Memory — L072']),
 ('Write — B-input','Write — L102',(10.2,0),(10.2,16),1,-1,22.5,['Write — L102'])]:
 sa,sb=proj(a),proj(b);angles=np.arange(0,90,2.5);aa=np.array([affinity.rotate(sa,float(ra*t),origin=ca) for t in angles],dtype=object);best=None
 for off in np.arange(0,pitch,.25):
  bb=np.array([affinity.rotate(sb,float(rb*t+off),origin=cb) for t in angles],dtype=object);areas=shapely.area(shapely.intersection(aa,bb));cost=(float(max(areas)),float(sum(areas)),float(off))
  if best is None or cost<best:best=cost
 for n in group:
  T=trimesh.transformations.rotation_matrix(np.radians(best[2]),[1,0,0],[0,*cb]);A[n]=trimesh.transform_points(A[n],T)
 report.append(dict(pair=[a,b],rotated_together=group,offset_deg=best[2],maximum_projected_overlap_mm2=best[0],samples=len(angles),speed_ratio=[ra,rb]));print(report[-1],flush=True)
arr=[]
for p in hs:p['offset']=sum(a.size for a in arr);arr.append(A[p['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr));(O/'hardware.json').write_text(json.dumps(hs,indent=2));(O/'Connected gear phases.json').write_text(json.dumps(report,indent=2))
