"""Separate collar-clearance audit; friction ribs are intentionally excluded."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Functional register'
P=json.load(open(O/'parts.json'));V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def verts(p):return V[p['offset']//3:p['offset']//3+p['vertices']]
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
scene=[(p,solid(verts(p))) for p in P if p['kind']=='printed'];rows=[]
for p in P:
 if p.get('lego_part') not in ['2780','6558']:continue
 a=verts(p);axis=p.get('axis',int(np.argmax(np.ptp(a,axis=0))));c=(a.min(0)+a.max(0))/2;radial=[i for i in range(3) if i!=axis];r=np.linalg.norm(a[:,radial]-c[radial],axis=1);collar=a[r>2.7]
 if not len(collar):continue
 lo,hi=collar[:,axis].min(),collar[:,axis].max();radius=r.max()
 # Conservative unsplit collar envelope, excluding shank/rib friction.
 s=m.Manifold.cylinder(hi-lo,radius,circular_segments=64)-m.Manifold.cylinder(hi-lo,2.7,circular_segments=64)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 cc=c.copy();cc[axis]=lo;s=s.translate(cc);bb=np.array(s.bounding_box()).reshape(2,3);hits=[]
 for q,t in scene:
  qb=np.array(t.bounding_box()).reshape(2,3)
  if np.any(np.minimum(bb[1],qb[1])-np.maximum(bb[0],qb[0])<.001):continue
  vol=(s^t).volume()
  if vol>.025:hits.append(dict(part=q['id'],volume_mm3=float(vol)))
  # A smaller smooth-core envelope excludes intentional friction ribs,
 # but detects material accidentally filling the nominal through hole.
 core=m.Manifold.cylinder(float(a[:,axis].max()-a[:,axis].min()),2.35,circular_segments=32)
 if axis==0:core=core.rotate([0,90,0])
 if axis==1:core=core.rotate([-90,0,0])
 cc=c.copy();cc[axis]=a[:,axis].min();core=core.translate(cc);cb=np.array(core.bounding_box()).reshape(2,3);shank_hits=[]
 for q,t in scene:
  qb=np.array(t.bounding_box()).reshape(2,3)
  if np.any(np.minimum(cb[1],qb[1])-np.maximum(cb[0],qb[0])<.001):continue
  vol=(core^t).volume()
  if vol>.025:shank_hits.append(dict(part=q['id'],volume_mm3=float(vol)))
 row=dict(pin=p['id'],axis=axis,collar_interval_mm=[float(lo),float(hi)],collar_radius_mm=float(radius),hits=hits,shank_hits=shank_hits);rows.append(row)
 if hits or shank_hits:print(row,flush=True)
(O/'Pin collar checks.json').write_text(json.dumps(dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),pins=rows,collar_clearance_pass=not any(r['hits'] for r in rows),pin_clearance_pass=not any(r['hits'] or r['shank_hits'] for r in rows),scope=__doc__,physical_validation=False),indent=2))
print('COLLARS',len(rows),'FAIL',sum(bool(r['hits'] or r['shank_hits']) for r in rows),flush=True)
