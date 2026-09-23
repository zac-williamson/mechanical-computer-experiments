from geometry import *
from clean_print_mesh import clean
O=OUT/'Flat-actuator';D=O/'Two-piece exploration'
base=solid(trimesh.load(D/'B removed candidate.stl'))
# Print-view -X is assembly +Z. Extend the fork back 1.5 mm, retaining its
# existing 3.2 mm axial width and avoiding the clutch's rotating envelope.
root=box([-1.6,7.2,7.45],[1.6,11.6,9.0])
s=(base+root).simplify(.005);t=clean(mesh(s));p=D/'Reinforced fork candidate.stl';t.export(p,file_type='stl_ascii');t=clean(trimesh.load(p));assert t.is_watertight and len(t.split())==1;t.export(p,file_type='stl_ascii')
t.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]));t.apply_translation(-t.bounds[0]);t.export(D/'Reinforced fork candidate - print.stl',file_type='stl_ascii');mask=(t.face_normals[:,2]<-.72)&(t.triangles_center[:,2]>.05)
r=dict(back_extension_mm=1.5,axial_width_mm=3.2,added_mm3=(s-base).volume(),steep_underside_mm2=float(t.area_faces[mask].sum()),watertight=True,components=1);(D/'Fork reinforcement geometry.json').write_text(json.dumps(r,indent=2));print(r)
