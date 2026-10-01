from pathlib import Path
import json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'fork-seam-candidate'
d=json.loads((O/'Clearance checks.json').read_text());assert not any(d[k] for k in ['printed_interferences','hardware_interferences','band_interferences']);assert json.loads((O/'Tiling checks.json').read_text())['passed']
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cx(r,a,b,y,z):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,y,z])
t=trimesh.load_mesh(O/'Carriage body.stl');new=solid(t);old=solid(trimesh.load_mesh(O/'baseline/Carriage body.stl'));delta=(old-new)+(new-old)
regions={'clutch contact':cx(6.8,-1.599,1.599,10.2,0),'worm bearing':cx(5.1,9.101,16.249,10.2,16),'locking detents':box([-12,23,35.7],[3,30,39.5]),'rod pin mounting':box([6,16.5,27],[16.25,22.4,37])}
changes={n:(delta^s).volume() for n,s in regions.items()};assert all(v<.001 for v in changes.values()),changes
for z in [9.3,11.3]:assert (new^box([1.61,3.76,z-.01],[6.9,10.49,z+.01])).volume()<1e-9
r=json.loads((O/'Fork seam changes.json').read_text());assert r['fork_root_intersection_mm3']>0 and r['rib_root_intersection_mm3']>0
p=trimesh.load_mesh(O/'Carriage body print.stl');s=solid(p);islands=[]
for h in np.arange(.16,p.extents[2],.16):
 prev=s.slice(h-.16)
 for c in s.slice(h).decompose():
  if c.area()>.1 and (c^prev.offset(.161,m.JoinType.Round,circular_segments=64)).area()<.01:islands.append(float(h))
assert not islands,islands
r.update(preserved_contact_regions_mm3=changes,watertight=bool(t.is_watertight),solid_count=len(t.split()),unsupported_island_layers=islands,layer_sample_mm=.16,sliver_regions_empty=True)
(O/'Fork seam checks.json').write_text(json.dumps(r,indent=2));print(r,flush=True)
# Update the separately linked carriage-only plate too.
a=[];cursor=0
for n in ['Carriage body','Carriage bearing end']:
 q=trimesh.load_mesh(O/(n+' print.stl'));q.apply_translation(-q.bounds[0]);q.apply_translation([cursor,0,0]);cursor+=q.extents[0]+8;a.append(q)
trimesh.util.concatenate(a).export(O/'Carriage print layout.stl',file_type='stl_ascii')
assembly=(R/'fork-brace/assembly.py').read_text().replace('fork-brace-candidate','fork-seam-candidate')
exec(compile(assembly,'carriage insertion validation','exec'),{'__file__':str(R/'fork-brace/assembly.py')})
runpy.run_path(str(R/'fork-seam-fix/render.py'),run_name='__main__')
