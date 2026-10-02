from geometry import *
from clean_print_mesh import clean
O=OUT/'Flat-actuator';D=O/'Two-piece exploration';base=solid(trimesh.load(D/'Joined fork candidate.stl'))
# Close the 1.8 mm Y gap below the left bearing support, down to its rear web.
s=base+box([-15.6,23.99,13.39],[-8,25.81,18]);p=D/'Filled bearing notch candidate.stl';t=clean(mesh(s));t.export(p,file_type='stl_ascii');t=clean(trimesh.load(p));assert t.is_watertight and len(t.split())==1;t.export(p,file_type='stl_ascii');print('net added mm3',(s-base).volume())
