from common import *
s=load('Left carriage half')
joint=box([-15.59,-18.,28.01],[-8.01,-14.01,43.19])
assert (joint-s).volume()<.001, 'Roof-pillar load path is interrupted'
hub=cyl(6.5,20.2,35.8)
for name in ['Left carriage half','Right carriage half']:
    s=load(name)
    for q in np.linspace(-4.6,4.575,185):
        assert (s.translate([float(q),0,0])^hub).volume()<.001
print('Roof joint and rotating hub envelope checks passed.')
