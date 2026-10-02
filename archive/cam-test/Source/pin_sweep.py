from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from motion import transform
ROOT=Path(__file__).resolve().parent;import os;O=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));D=json.loads((O/'Assembly manifest.json').read_text())
def solid(t):return m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
cam=solid(trimesh.load(O/'Write diagonal cam.stl'))
# Conservative outer pin cylinder includes split shaft and friction ribs.
core=m.Manifold.cylinder(16,2.61,circular_segments=48,center=True).rotate([0,90,0]);collar=m.Manifold.cylinder(.8,3.2,circular_segments=48,center=True).rotate([0,90,0]);pin=core+collar
fixed={r['record_id']:pin.transform(np.column_stack([np.array(r['matrix']).reshape(3,3),np.array(r['pos'])*.4])) for r in D['records'] if r['part']=='2780.dat' and r['motion']=='fixed'}
hits={}
for q in np.linspace(-4.35,4.325,181):
 c=cam.translate([-q,0,0]);cb=np.asarray(c.bounding_box())
 for name,s in fixed.items():
  b=np.asarray(s.bounding_box())
  if np.any(np.minimum(cb[3:],b[3:])-np.maximum(cb[:3],b[:3])<=0):continue
  v=(c^s).volume()
  if v>1e-4:hits[name]=max(v,hits.get(name,0))
result=dict(samples=181,fixed_pins=len(fixed),shaft_radius_mm=2.61,collar_radius_mm=3.2,cam_intersections_mm3=hits)
(O/'Fixed pin sweep.json').write_text(json.dumps(result,indent=2));print(result);assert not hits
