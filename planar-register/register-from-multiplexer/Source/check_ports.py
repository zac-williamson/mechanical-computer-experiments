from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());rows=[]
for name,y,z in [('WRITE',10.2,32),('D',10.2,0)]:
 s=m.Manifold.cylinder(16,4.1,circular_segments=64).rotate([0,90,0]).translate([132.8,y,z]);hits=[]
 for p in meta:
  t=trimesh.load(O/(p['id']+'.stl'));a=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
  for q in ([-4.6,0,4.6] if p['motion']=='carriage' else [0]):
   v=float((s^a.translate([q,0,0])).volume())
   if v>.005:hits.append(dict(part=p['id'],q=q,overlap_mm3=v))
 rows.append(dict(port=name,coupler_envelope=[132.8,148.8],radius_mm=4.1,printed_intersections=hits))
(O/'Port access checks.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
