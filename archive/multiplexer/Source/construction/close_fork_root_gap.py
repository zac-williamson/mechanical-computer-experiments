from geometry import *
from clean_print_mesh import clean
O=OUT/'Flat-actuator';D=O/'Two-piece exploration'
base=solid(trimesh.load(D/'Uniform arm candidate.stl'))
# The fork has a flat back at Z=7.5. The prior R7.6 subtraction wrongly
# cut a 0.1 mm crescent out of this joint. Overlap the flat back by 0.05 mm.
s=base+box([-1.6,7.2,7.45],[1.6,11.6,9.0]);p=D/'Joined fork candidate.stl';t=clean(mesh(s));t.export(p,file_type='stl_ascii');t=clean(trimesh.load(p));assert t.is_watertight and len(t.split())==1;t.export(p,file_type='stl_ascii')
t.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]));t.apply_translation(-t.bounds[0]);t.export(D/'Joined fork candidate - print.stl',file_type='stl_ascii');print('added mm3',(s-base).volume())
# Check a continuous filled joint through its full nominal axial width.
probe=box([-1.59,7.21,7.49],[1.59,11.39,7.61]);missing=(probe-s).volume();print('missing joint volume',missing);assert missing<1e-6
