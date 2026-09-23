from geometry import *
from clean_print_mesh import clean
O=Path(__file__).resolve().parent/'Flat-actuator';D=O/'Two-piece exploration';D.mkdir(exist_ok=True)
meta=json.loads((O/'printed-parts.json').read_text())
def read(n):
 t=trimesh.load(O/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
P={p['id']:read(p['id']) for p in meta};fixed={p['id']:P[p['id']] for p in meta if p['motion']=='fixed'}
body=P['Carriage fork and roof'];results=[]
for side in ['Left','Right']:
 name=side+' carriage bearing support';a,b=(-15.6,-7.6) if side=='Left' else (7.6,15.6)
 # Extend the bearing's rear web along Z, then join the existing roof.
 extension=box([a,25.8,18],[b,29.8,43.7995505])+box([a,11.85,41.4],[b,29.8,43.7995505])
 # Root the existing body's planar -X surfaces on the same bed as the left bearing.
 if side=='Left':
  t=mesh(body);mask=(t.face_normals[:,0]<-.99)&(abs(t.triangles_center[:,0]+7.8)<.01)
  profiles=[Polygon(tri[:,[1,2]]) for tri in t.triangles[mask]]
  profile=unary_union(profiles)
  extension+=extr(profile,0,7.81).rotate([0,90,0]).rotate([90,0,0]).translate([-15.6,0,0])
 # Conservative rectangular cheek notches include full travel and 0.4 mm clearance.
 for n in ['Front bearing cheek','Rear bearing cheek']:
  t=trimesh.load(O/(n+'.stl'));lo,hi=t.bounds
  lo=lo-np.array([4.575+.4,.4,.4]);hi=hi+np.array([4.6+.4,.4,.4]);extension-=box(lo,hi)
 s=body+P[name]+extension
 # Preserve both existing joint passages when extending across the former seams.
 for y,z in [(24.,2.),(29.6,18.)]:
  s-=cyl(2.45,-17,17,0,(0,y,z))
  for joint in [-8.2,8.2]:
   s-=cyl(3.45,joint-.95,joint+.95,0,(0,y,z))
   for xa,ra,rb in [(joint-1.95,2.45,3.45),(joint+.95,3.45,2.45)]:
    s-=m.Manifold.cylinder(1.,ra,rb,circular_segments=96).rotate([0,90,0]).translate([xa,y,z])
 collisions=[]
 for q in np.linspace(-4.6,4.575,185):
  for n,f in fixed.items():
   v=(s.translate([q,0,0])^f).volume()
   if v>.001:collisions.append(dict(part=n,q=float(q),mm3=v))
 t=clean(mesh(s));t.export(D/(side+' merged candidate.stl'))
 orientations=[]
 for sign in [-1,1]:
  pr=t.copy();pr.apply_transform(trimesh.transformations.rotation_matrix(sign*np.pi/2,[0,1,0]));pr.apply_translation(-pr.bounds[0]);down=(pr.face_normals[:,2]<-.72)&(pr.triangles_center[:,2]>.05)
  orientations.append(dict(rotation=sign*90,steep_underside_mm2=float(pr.area_faces[down].sum()),height=float(pr.extents[2])))
  pr.export(D/(side+' merged candidate '+str(sign*90)+' - print.stl'))
 results.append(dict(side=side,solids=len(s.decompose()),watertight=t.is_watertight,added_mm3=extension.volume(),fixed_collisions=collisions,orientations=orientations))
(D/'feasibility.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
