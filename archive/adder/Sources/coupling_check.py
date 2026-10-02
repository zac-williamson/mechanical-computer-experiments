from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1];D=json.loads((O/'Assembly manifest.json').read_text())
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
parts=[(p,solid(trimesh.load(O/p['path']))) for p in D['prints']]
hits=[]
for q in [-4.35,0,4.325]:
 for r in D['records']:
  if not r['record_id'].startswith('Shared carriage coupling pin'):continue
  p=np.array(r['pos'])*.4+[q,0,0]
  core=m.Manifold.cylinder(16,2.39,circular_segments=48,center=True)
  flange=m.Manifold.cylinder(.7,3.15,circular_segments=48,center=True)
  pin=(core+flange).translate(p.tolist())
  for d,s in parts:
   if d['motion']=='carriage' and d['actor'] in ['C','S']:s=s.translate([q,0,0])
   aa=np.array(pin.bounding_box());bb=np.array(s.bounding_box())
   if np.any(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])<.001):continue
   v=(pin^s).volume()
   if v>.02:hits.append([q,r['record_id'],d['id'],v])
(O/'Coupling pin checks.json').write_text(json.dumps(dict(positions=3,pins=4,friction_ribs_excluded=True,hits=hits),indent=2))
print('Coupling pin hits',hits)
