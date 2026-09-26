"""Check cap-off insertion corridors, reinforced mounting sections and retained envelope."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'Wall register'
P={p['id']:p for p in json.loads((O/'parts.json').read_text())};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def solid(n):
 p=P[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def box(lo,hi):return m.Manifold.cube(np.array(hi)-lo).translate(lo)
def cyl(r,lo,hi,c):return m.Manifold.cylinder(hi-lo,r,circular_segments=32).rotate([-90,0,0]).translate([c[0],lo,c[2]])
guides=[]
for e in json.loads((O/'Rod guide assembly schedule.json').read_text())['connections']:
 x,y,z=e['guide_centre_mm'];h=e['guide_half_length_mm']
 host=solid(e['host'])
 if e['control']=='write':
  # Place the shank in the gap behind the CLOCK hardware from the left,
  # then push it rearwards into the uncapped WRITE guide.
  entry_y=y-8.2
  if e['module']=='control' and z==-204:
   side=box([x-3.1,entry_y,z-h+.05],[P[e['host']]['bounds'][1][0]+10,entry_y+6,z+h-.05])
  else:side=box([P[e['host']]['bounds'][0][0]-10,entry_y,z-h+.05],[x+3.1,entry_y+6,z+h-.05])
  corridor=side+box([x-3.1,entry_y,z-h+.05],[x+3.1,y+6.1,z+h-.05])
 else:corridor=box([x-3.1,y-25,z-h+.05],[x+3.1,y+6.1,z+h-.05])
 occupied=(corridor^host).volume()
 guides.append(dict(part=e['part'],host=e['host'],insertion_corridor_occupied_mm3=occupied,pins=len(e['pins']),pass_check=occupied<.001 and len(e['pins'])>=2))
mounts=[]
for e in json.loads((O/'Frame fixture schedule.json').read_text())['fixtures']:
 s=solid(e['part']);y=e['interface_y_mm']
 for pin in e['fasteners']:
  c=pin['centre_mm'];sections=[]
  for lo,hi in [(y-.35,y-.25),(y+.05,y+.15)]:
   ring=cyl(4.7,lo,hi,c)-cyl(3.35,lo-.01,hi+.01,c);sections.append((ring^s).volume()/ring.volume())
  mounts.append(dict(part=e['part'],pin=pin['part'],section_material_fraction=sections,pass_check=min(sections)>.99))
limits={'bit':np.array([[-141.8,-12.320086537,-47.8],[166.2,48.5,64.4]]),'control':np.array([[-173.8,-18.463680077,-224],[-84,56.5,-48.2]])};bounds=[]
for mod,limit in limits.items():
 ps=[p for p in P.values() if p['module']==mod];b=np.array([[min(p['bounds'][0][k] for p in ps) for k in range(3)],[max(p['bounds'][1][k] for p in ps) for k in range(3)]])
 bounds.append(dict(module=mod,bounds_mm=b.tolist(),pass_check=bool(np.all(b[0]>=limit[0]-.001) and np.all(b[1]<=limit[1]+.001))))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),guide_corridors=guides,mount_sections=mounts,module_envelopes=bounds,manufacturing_revision_pass=all(r['pass_check'] for r in guides+mounts+bounds),scope=__doc__,limitations=['Guide corridor proves the rod shank can enter an uncapped guide laterally; install guides/rods before obstructing followers and actuator hardware. It is not an exhaustive full-assembly path proof.','Mount section checks verify material, not printed strength.'])
(O/'Manufacturing revision checks.json').write_text(json.dumps(report,indent=2));print('Manufacturing revision',report['manufacturing_revision_pass']);print([r for r in guides+mounts+bounds if not r['pass_check']])
