from geometry import *
from clean_print_mesh import clean
O=OUT/'Flat-actuator';D=O/'Two-piece exploration'
base=solid(trimesh.load(D/'Simplified roof candidate.stl'))
# Remove the lower-print-height thin gear-relief web, rather than filling its
# concave surface. Preserve the worm bearing, the rear guide and the fork root.
cut=box([-15.61,11.39,3.39],[-7.3,20.2,9.001])
s=(base-cut).simplify(.005)
t=clean(mesh(s));t.export(D/'B removed candidate.stl',file_type='stl_ascii');t=clean(trimesh.load(D/'B removed candidate.stl'));print('watertight',t.is_watertight,'components',[(x.volume,len(x.faces)) for x in t.split()]);assert t.is_watertight and len(t.split())==1;t.export(D/'B removed candidate.stl',file_type='stl_ascii')
t.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]));t.apply_translation(-t.bounds[0]);t.export(D/'B removed candidate - print.stl',file_type='stl_ascii');mask=(t.face_normals[:,2]<-.72)&(t.triangles_center[:,2]>.05)
r=dict(removed_mm3=(base-s).volume(),added_mm3=(s-base).volume(),steep_underside_mm2=float(t.area_faces[mask].sum()),watertight=t.is_watertight,components=1);(D/'B removal geometry.json').write_text(json.dumps(r,indent=2));print(r)
