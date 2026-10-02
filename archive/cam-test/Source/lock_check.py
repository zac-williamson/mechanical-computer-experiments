from pathlib import Path
import sys,json,math
import numpy as np,trimesh,manifold3d as m
from shapely.geometry import Point,LineString
from shapely.ops import polygonize,unary_union
from motion import transform,lift
from band import lock_band
ROOT=Path(__file__).resolve().parent;import os;O=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));D=json.loads((O/'Assembly manifest.json').read_text());M={p['id']:trimesh.load(O/p['path']) for p in D['prints']};meta={p['id']:p for p in D['prints']}
cam=M['Write diagonal cam'];sec=cam.section(plane_origin=[0,-27,0],plane_normal=[0,1,0]);polys=list(polygonize([LineString(v[:,[0,2]]) for v in sec.discrete]));material=max(polys,key=lambda p:p.area)
rows=[]
for q in np.linspace(-4.35,4.325,71):
 x=-7+q;good=[]
 for z in np.arange(12.,18.401,.01):
  if Point(x,z).buffer(2.46,quad_segs=32).intersection(material).area<1e-6:good.append(float(z))
 assert good,('Follower jam',q)
 # The larger native pin rib envelope must also fit at the commanded position.
 assert Point(x,18+lift(q)).buffer(2.6001,quad_segs=32).intersection(material).area<.001,('Native rib jam',q)
 rows.append(dict(q=float(q),withdrawal_from_hold=float(q+4.35),min_follower_Z=min(good),max_follower_Z=max(good),min_toe_engagement_mm=min(good)+14-28,max_toe_engagement_mm=max(good)+14-28))
# Positive engagement throughout a 2 mm disturbance in write-carriage position.
protected=[r for r in rows if r['withdrawal_from_hold']<=2]
assert min(r['min_toe_engagement_mm'] for r in protected)>1.5
# Fully released before first possible clutch contact at q+1.2, using actual clearance envelope.
released=[r for r in rows if r['q']>=1.2];assert max(r['max_toe_engagement_mm'] for r in released)<-.5
hits={}
def solid(t):return m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
for q in np.linspace(-4.35,4.325,19):
 band=lock_band(q);bs=solid(band)
 for kq in [-4.35,4.325]:
  st={'K':dict(q=kq,b=14 if kq<0 else -14.75,w=0,g=0,offset=0),'W':dict(q=q,b=0,w=0,g=0,offset=0),'E':dict(q=4.325,b=-14.75,w=0,g=0,offset=0)}
  for name,t in M.items():
   p=meta[name];tt=t.copy().apply_transform(transform(p['actor'],p['motion'],st,D['actors']))
   if np.any(np.minimum(band.bounds[1],tt.bounds[1])-np.maximum(band.bounds[0],tt.bounds[0])<=0):continue
   v=(bs^solid(tt)).volume()
   if v>.01:hits[name]=max(hits.get(name,0),v)
print('Band hits',hits)
out=dict(method='Cam section at Y=-27, 2.46 mm radius conservative clearance envelope plus 2.6001 mm native rib fit check; elastic bias chooses the engaged side of its clearance',minimum_engagement_during_2mm_disturbance=min(r['min_toe_engagement_mm'] for r in protected),minimum_clearance_at_clutch_contact=-max(r['max_toe_engagement_mm'] for r in released),band_printed_intersections_mm3=hits,samples=rows)
(O/'Lock checks.json').write_text(json.dumps(out,indent=2));print({k:v for k,v in out.items() if k!='samples'})
