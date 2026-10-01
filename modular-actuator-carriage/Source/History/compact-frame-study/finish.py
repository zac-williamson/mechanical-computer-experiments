from pathlib import Path
import json
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'compact-frame-candidate'
a=json.loads((O/'Clearance checks.json').read_text());t=json.loads((O/'Tiling checks.json').read_text())
assert not a['printed_interferences'] and not a['hardware_interferences'] and not a['band_interferences'], 'Mechanism collisions'
assert t['passed'],'Tiling or assembly failed'
# Bridge bed faces are broad Y faces; bores print vertically.
layout=[];x=0
for n,count in [('Horizontal frame bridge',2),('Vertical frame bridge',2),('Centre frame bridge',1)]:
 q=trimesh.load_mesh(O/(n+'.stl'));q.apply_transform(trimesh.geometry.align_vectors([0,1,0],[0,0,-1]));q.apply_translation(-q.bounds[0]);q.export(O/(n+' print.stl'),file_type='stl_ascii')
 for _ in range(count):
  p=q.copy();p.apply_translation([x,0,0]);layout.append(p);x+=q.extents[0]+5
trimesh.util.concatenate(layout).export(O/'2x2 frame connectors print layout.stl',file_type='stl_ascii')
checks=[]
for n in ['Module base','Left bearing wall','Right bearing wall','Carriage control rod','Lock control rod']:
 q=trimesh.load_mesh(O/(n+' print.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(q.vertices),np.ascontiguousarray(q.faces,dtype=np.uint64)))
 islands=[]
 for h in np.arange(.24,q.extents[2],.24):
  current=s.slice(h);prev=s.slice(h-.24)
  for c in current.decompose():
   if c.area()>.1 and (c^prev.offset(.241,m.JoinType.Round,circular_segments=64)).area()<.01:islands.append(float(h))
 checks.append(dict(part=n,watertight=q.is_watertight,solids=len(q.split()),unsupported_island_layers=islands))
(O/'Compact print checks.json').write_text(json.dumps(checks,indent=2));print('PRINT',checks,flush=True)
notes='''# Compact module candidate

Base footprint: 72 mm X × 64 mm Z, X = −31.5 to +40.5, Z = −8 to +56. Base remains Y = 32 to 48.4; no increase in depth. Moving rod ends extend beyond the base to mate with adjoining modules.

The frame-joining holes remain at Z = 0 and 48, now X = −26 and +35. Only the left bearing-wall pins move along Z, to 10 and 38. The left wall has rear access gaps so connecting bridges can be pressed into place after the walls are mounted. Pin ends remain flush with the base underside at Y = 48.4.

Both rods are shortened 4 mm at each end, carrying their existing lap joints and holes inward. The actuator axle is 9L; the output axles are 4L each. Four half-bush retainers sit 0.8 mm farther inward, in shallow bearing recesses, leaving 0.2 mm to the external connectors. The 72 mm nominal pitch gives 8 mm nominal insertion into each end of a 2L connector. Exact physical seating against the smooth connector centre stop remains a fit test; the reference mesh has a 0.2 mm divider, and keyed mating contacts are excluded from clearance testing.

Checked: operating geometry, 2×2 neighbouring modules with independent rows, bridge and rod-pin insertion, watertight printable components. These are geometric tests, not loaded-operation or physical print validation. Use the updated base, both walls, both rods and frame bridges together; do not mix their positions with the 80 mm version. The carriage, actuator cheeks, bolt and guide geometry are unchanged.
'''
(O/'Compact module notes.md').write_text(notes)
h=(O/'Viewer.html').read_text().replace('Candidate under validation.','Geometry and connector insertion checked; physical fit testing remains.').replace('<p><a href="Lock%20guide', '<p><a href="Compact%20module%20notes.md">72 mm module notes</a> · <a href="Lock%20guide');tag='<script type="application/json" id="data">';pre,r=h.split(tag,1);raw,post=r.split('</script>',1);v=json.loads(raw);v.setdefault('labels',{}).update({'Bearing wall pin -24.4 / 0':'Left bearing-wall pin at Z = 10 mm','Bearing wall pin -24.4 / 50':'Left bearing-wall pin at Z = 38 mm'});(O/'Viewer.html').write_text(pre+tag+json.dumps(v,separators=(',',':'))+'</script>'+post)
print('Compact candidate checks complete',flush=True)
