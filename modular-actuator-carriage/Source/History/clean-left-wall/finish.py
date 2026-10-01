from pathlib import Path
import json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'clean-left-wall-candidate'
d=json.loads((O/'Clearance checks.json').read_text());assert not any(d[k] for k in ['printed_interferences','hardware_interferences','band_interferences']);assert json.loads((O/'Tiling checks.json').read_text())['passed']
t=trimesh.load_mesh(O/'Left bearing wall print.stl');s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)));islands=[]
for h in np.arange(.16,t.extents[2],.16):
 p=s.slice(h-.16);q=s.slice(h)
 for c in q.decompose():
  if c.area()>.1 and (c^p.offset(.161,m.JoinType.Round,circular_segments=64)).area()<.01:islands.append(float(h))
assert not islands,islands
# New geometry has a broad continuous web, while retaining the mounting bores.
r=dict(watertight=bool(t.is_watertight),solid_count=len(t.split()),unsupported_island_layers=islands,layer_sample_mm=.16,main_wall_thickness_X_mm=4,mounting_foot_thickness_X_mm=8,lower_axle_boss_outer_radius_mm=7,bearing_bore_radius_mm=2.65,retainer_recess_radius_mm=3.85,lower_boss_minimum_radial_material_mm=3.15,construction='New extruded YZ profile, circular bearing boss and two solid feet. Functional guide and pin holes generated parametrically. No original wall solid used.',limits='Geometric checks only; print and mechanical load testing remain necessary.')
(O/'Left wall checks.json').write_text(json.dumps(r,indent=2));print(r,flush=True)
notes=(O/'Compact module notes.md').read_text();notes+='\n## Rebuilt left bearing wall\n\nA new continuous 4 mm-thick web joins both mounting feet. The former central rear cutout is filled. The lower axle uses a 7 mm-radius circular boss with an angled shoulder into the wall; minimum radial material at its retainer recess is 3.15 mm. Pin centres, rod-guide clearances and the +X printing face remain unchanged. Rear access above and below the feet remains open for the frame bridges. The lower actuator bearing, rod guides and all mounting positions were checked against the moving module and tiled neighbours.\n';(O/'Compact module notes.md').write_text(notes)
runpy.run_path(str(R/'clean-left-wall/render.py'),run_name='__main__')
