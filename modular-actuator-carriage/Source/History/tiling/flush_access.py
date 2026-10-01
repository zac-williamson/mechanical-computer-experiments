from pathlib import Path
import json,gzip,base64
import numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'tile-candidate'
t=trimesh.load(O/'Module base.stl');s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
checks=[]
for x in [-34,34]:
 for z in [0,48]:
  tool=m.Manifold.cylinder(8.8,2.3,circular_segments=32).rotate([-90,0,0]).translate([x,40.3,z]);vol=(tool^s).volume()
  checks.append(dict(hole=[x,z],open_to_rear=vol<1e-6,obstructing_mm3=vol))
T=json.loads((O/'Tiling geometry.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(T['geometry'])),dtype='<f4').reshape(-1,3)
pins=[]
for p in T['parts']:
 if not p['name'].startswith('Frame bridge pin'):continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']];end=float(a[:,1].max());pins.append(dict(pin=p['name'],rear_Y=end,flush=abs(end-48.4)<1e-4))
r=dict(passed=all(x['open_to_rear'] for x in checks) and len(pins)==12 and all(p['flush'] for p in pins),bottom_plane_Y=48.4,holes=checks,pins=pins,removal_direction='Push from +Y toward -Y. The pin collar pushes the bridge off its seat; remove the bridge before separating a pin from it.')
(O/'Rear pin access checks.json').write_text(json.dumps(r,indent=2));assert r['passed'],r
print('Rear access: all four socket bores open; all twelve frame pin ends flush with Y=48.4.',flush=True)
