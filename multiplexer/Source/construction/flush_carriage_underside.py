from geometry import *
from clean_print_mesh import clean
O=OUT/'Flat-actuator'; D=O/'Two-piece exploration'
base=solid(trimesh.load(D/'Clean arm candidate.stl'))
# Remove only the residual 0.2 mm ledge beneath the front joining-pin block.
cut=box([-15.61,27.799999,-1.81],[7.81,28.01,5.80001])
s=base-cut
p=D/'Flush underside candidate.stl'
t=clean(mesh(s));t.export(p,file_type='stl_ascii')
t=clean(trimesh.load(p));assert t.is_watertight and len(t.split())==1
t.export(p,file_type='stl_ascii')
assert (s-base).volume()<1e-6
assert (s^box([-15.59,27.80001,3.41],[7.79,28.001,5.79])).volume()<1e-6
report=dict(removed_mm3=(base-s).volume(),added_mm3=(s-base).volume(),watertight=t.is_watertight,solid_count=len(t.split()),underside_y_mm=27.8)
t.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]));t.apply_translation(-t.bounds[0]);t.export(D/'Flush underside candidate - print.stl',file_type='stl_ascii')
(D/'Flush underside checks.json').write_text(json.dumps(report,indent=2));print(report)
