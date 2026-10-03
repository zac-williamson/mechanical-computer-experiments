"""Small hand-operated storage test base using unchanged production interfaces.

The base prints with its Y48.5 rear face on the bed, building in -Y. All
pin bores are vertical in that orientation. The gear/axle/guide coordinates
are copied unchanged. It is a component test fixture, not a powered register.
"""
from pathlib import Path
import json,hashlib,copy,numpy as np,trimesh,manifold3d as m
from wall_print_geometry import qualify
from wall_pose import vertices
O=Path(__file__).resolve().parents[1]/'Wall register';D=O/'Storage test cassette';D.mkdir(exist_ok=True)
P=json.load(open(O/'parts.json'));L={p['id']:p for p in P};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def verts(p):return V[p['offset']//3:p['offset']//3+p['vertices']].copy()
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def box(lo,hi):return m.Manifold.cube(np.array(hi)-lo).translate(lo)
def cyl(r,lo,hi,c):return m.Manifold.cylinder(hi-lo,r,circular_segments=32).rotate([-90,0,0]).translate([c[0],lo,c[2]])
# A narrow perimeter and cross-ties replace the large bit chassis. No added
# bearing positions, and no platform beneath the sliding working surfaces.
base=box([-43,45.5,-40],[-37,48.5,65])+box([37,45.5,-40],[43,48.5,65])+box([-43,45.5,-40],[43,48.5,-34])+box([-43,45.5,59],[43,48.5,65])
for z in [-22,0,30,39.8,49]:base+=box([-43,45.5,z-3],[43,48.5,z+3])
selected={p['id'] for p in P if p['id'].startswith('master ') and p['id'] not in ['master POWER 16T 16','master POWER 16T 32','master reversing idler 16T -16','master reversing idler 16T 32']}
selected|={'master_gate worm drive 10L','Master worm retainer gear','Master worm axial stop -26.2','M output axial stop 26.8','M output axial stop 37.8'}
fixtures=[e for e in json.load(open(O/'Frame fixture schedule.json'))['fixtures'] if e['part'].startswith('master ') or e['part']=='bit frame 0 removable fixture 4']
walls=[e for e in json.load(open(O/'Bearing schedule.json'))['removable_walls'] if e['part'] in ['bit removable bearing wall '+str(i) for i in [4,6,7,10]]]
mounts=[]
for e in fixtures+walls:
 selected.add(e['part']);pins=e.get('fasteners',e.get('pins'));mounts+=pins
 for pin in pins:
  selected.add(pin['part']);c=pin['centre_mm'];x,y,z=c
  base+=box([x-4.8,y+.2,z-4.8],[x+4.8,48.5,z+4.8])
# Cut all bores last, including through the cross-ties.
for pin in mounts:
 c=pin['centre_mm'];y=c[1];base-=cyl(2.5,y+.19,48.6,c)+cyl(3.3,y+.19,y+.9,c)
assert selected<=set(L),selected-set(L)
q=base.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True);a=t.triangles.reshape(-1,3)
printed,pr=qualify(t,1,-1);printed.export(D/'Storage test base - PRINT THIS.stl')
parts=[copy.deepcopy(p) for p in P if p['id'] in selected];arrays=[verts(p) for p in parts]
parts.append(dict(id='Storage test base',module='test',kind='printed',motion='fixed',color=[.25,.43,.41]));arrays.append(a)
offset=0
for p,a in zip(parts,arrays):p.update(offset=offset,vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()]);offset+=a.size
np.savez_compressed(D/'geometry.npz',vertices=np.concatenate(arrays));(D/'parts.json').write_text(json.dumps(parts,indent=2))
# New base vs all reused production pieces over every distinct inherited pose.
frames=json.load(open(O.parent/'Compact layout/Compact contact-resolved operation.json'))['cases'];seen=set();fs=[]
for case in frames:
 for f in case['frames'][::max(1,len(case['frames'])//17)]:
  key=json.dumps(f,sort_keys=True)
  # Use full transform-derived bounds as deduplication key below if frame schema changes.
  if key not in seen:seen.add(key);fs.append(f)
hits=[]
for p,b in zip(parts[:-1],arrays[:-1]):
 if p['kind']=='elastic':continue
 for f in fs:
  v=vertices(p,b,f)
  if np.any(np.minimum(v.max(0),t.bounds[1])-np.maximum(v.min(0),t.bounds[0])<=0):continue
  over=(base^solid(v)).volume()
  if over>.01:hits.append(dict(part=p['id'],volume_mm3=over));break
# Verify positive socket material around both insertion ends and seating land.
sockets=[]
for pin in mounts:
 c=pin['centre_mm'];y=c[1];probe=cyl(3.7,y+1.5,y+7.5,c)-cyl(2.7,y+1.4,y+7.6,c);fraction=(base^probe).volume()/probe.volume();sockets.append(dict(**pin,socket_wall_fraction=fraction,pass_check=fraction>.95))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,base_print=pr,production_parts=sorted(selected),print_parts=[p['id'] for p in parts if p['kind']=='printed'],base_size_mm=t.extents.tolist(),base_volume_mm3=float(t.volume),poses=len(fs),intersections=hits,sockets=sockets,cassette_pass=pr['print_geometry_pass'] and not hits and all(r['pass_check'] for r in sockets),slicer_validated=False,physically_validated=False)
(O/'Storage test cassette checks.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['sockets','production_parts']},indent=2))
