from common import *
for p in ROOT.glob('* - print.stl'):
    t=trimesh.load(p)
    assert t.is_watertight and len(t.split())==1,p.name
    assert abs(t.bounds[0,2])<.001,p.name
p=ROOT/'Complete print layout.stl';t=trimesh.load(p)
assert t.is_watertight and len(t.split())==13
print('13 printable components: watertight, single solids, bed-aligned.')
