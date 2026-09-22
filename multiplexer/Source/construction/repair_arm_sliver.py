from geometry import *
from clean_print_mesh import clean
O=OUT/'Flat-actuator';D=O/'Two-piece exploration'
base=solid(trimesh.load(D/'Filled bearing notch candidate.stl'))
# Continue the existing A cross-section from just inside its back face in +Y.
# Slice in XZ, then extrude toward the carriage's Y=29.8 rear/base surface.
t=trimesh.load(D/'Reinforced fork candidate.stl');sec=t.section([0,1,0],[0,20.24,0]);profile=Polygon()
for ring in sec.discrete:
 p=Polygon(ring[:,[0,2]]).buffer(0);profile=profile.symmetric_difference(p)
profile=profile.intersection(Polygon([(-16,35),(27,35),(27,45),(-16,45)]))
extension=extr(profile,0,9.57).rotate([90,0,0]).translate([0,29.8,0])
s=(base-box([-16,20.24,39.79],[27,29.81,44])+extension).simplify(.005)
s=(s^box([-100,-100,35],[100,100,100]))+(base^box([-100,-100,-100],[100,100,35]))
p=D/'Clean arm candidate.stl';t=clean(mesh(s));t.export(p,file_type='stl_ascii');t=clean(trimesh.load(p));assert t.is_watertight and len(t.split())==1;t.export(p,file_type='stl_ascii')
t.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]));t.apply_translation(-t.bounds[0]);t.export(D/'Clean arm candidate - print.stl',file_type='stl_ascii');mask=(t.face_normals[:,2]<-.72)&(t.triangles_center[:,2]>.05)
print(dict(added_mm3=(s-base).volume(),steep_underside_mm2=float(t.area_faces[mask].sum()),watertight=True))

# Validate the serialized mesh too; remove precision debris after STL round-trip.
for path in [p,p.with_name(p.stem+" - print.stl")]:
 for _ in range(2):
  verified=clean(trimesh.load(path));assert verified.is_watertight and len(verified.split())==1;verified.export(path,file_type="stl_ascii")
 assert trimesh.load(path).is_watertight
