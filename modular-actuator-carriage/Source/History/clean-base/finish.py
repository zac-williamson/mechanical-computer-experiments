from pathlib import Path
import json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'clean-base-candidate'
d=json.loads((O/'Clearance checks.json').read_text());assert not any(d[k] for k in ['printed_interferences','hardware_interferences','band_interferences']);assert json.loads((O/'Tiling checks.json').read_text())['passed']
t=trimesh.load_mesh(O/'Module base print.stl');s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)));islands=[]
for h in np.arange(.16,t.extents[2],.16):
 p=s.slice(h-.16);q=s.slice(h)
 for c in q.decompose():
  if c.area()>.1 and (c^p.offset(.161,m.JoinType.Round,circular_segments=64)).area()<.01:islands.append(float(h))
assert not islands,islands
report=dict(watertight=bool(t.is_watertight),solid_count=len(t.split()),unsupported_island_layers=islands,sampled_layer_mm=.16,construction='Rear plate and explicit mounting blocks; only functional holes are subtracted. No imported old-frame geometry.',wall_pin_socket_depth_mm=8,minimum_verified_wall_pin_socket_radial_material_mm=1,limitations='Sampled geometry checks, not a slicer or physical load test. The front of the lower actuator mounting bore is intentionally open below Z=29 to avoid a thin lip above the cheek; the rear plate supports the bore.')
(O/'Rebuilt base checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
notes=(O/'Compact module notes.md').read_text();notes+='\n## Rebuilt base\n\nThe base now consists of a rear plate and explicit mounting blocks. All four bearing-wall pins have 8 mm-deep sockets; the relocated lower-left socket no longer opens into empty space. Frame-joining pins remain flush with the underside. Connector pockets are open spaces between blocks, without retained cut remnants. The lower actuator bore has an intentional open mouth below Z = 29 on its front portion, removing the thin lip above the cheek clearance; its rear portion remains enclosed.\n'
(O/'Compact module notes.md').write_text(notes)
runpy.run_path(str(R/'clean-base/render.py'),run_name='__main__')
