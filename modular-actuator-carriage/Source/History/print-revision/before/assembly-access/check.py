from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'adapted'
def load(n):
 t=trimesh.load(O/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=32)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
a=load('Carriage body');b=load('Carriage bearing end');rod=load('Carriage control rod')
import sys
if '--candidate' in sys.argv:
 opening=box([-17,-1,25],[17,14.4,40]);a-=opening;b-=opening
checks=[]
for shift in np.linspace(16,0,17):
 hit=rod.translate([0,-shift,0])^(a+b)
 checks.append(dict(step='Rod approach along +Y',offset=-shift,collision_mm3=hit.volume(),bounds=hit.bounding_box() if hit.volume()>.00001 else None))
for shift in np.linspace(16,0,33):
 hit=b.translate([shift,0,0])^a
 checks.append(dict(step='Right half approach along -X',offset=shift,collision_mm3=hit.volume(),bounds=hit.bounding_box() if hit.volume()>.00001 else None))
# A rod pin seated in the carriage exposes its other end toward -Y.
# Check the full insertion path at clearance diameter; radial pin friction is intentional.
for x in [-11,11]:
 for shift in np.linspace(16,0,17):
  pin=cy(2.45,6.3-shift,22.3-shift,1,[x,0,32])
  hit=pin^(a+b)
  checks.append(dict(step='Rod pin approach +Y',x=x,offset=-shift,collision_mm3=hit.volume(),bounds=hit.bounding_box() if hit.volume()>.00001 else None))
# Continuous conservative envelopes for the rod and pins (not just poses).
for label,envelope in [('Rod full straight insertion',box([-47.8,-9.8,28.2],[47.8,14.2,35.8]))]+[(f'Pin {x} full straight insertion',cy(2.45,-9.7,22.3,1,[x,0,32])+cy(3.25,-2.1,14.7,1,[x,0,32])) for x in [-11,11]]:
 hit=envelope^(a+b);checks.append(dict(step=label,collision_mm3=hit.volume(),bounds=hit.bounding_box() if hit.volume()>.00001 else None))
(Path(__file__).parent/('candidate.json' if '--candidate' in sys.argv else 'current.json')).write_text(json.dumps(checks,indent=2));print(json.dumps(checks,indent=2),flush=True)

if '--verify' in sys.argv:
 assert all(c['collision_mm3']<.02 for c in checks),[c for c in checks if c['collision_mm3']>=.02]
 (O/'Carriage assembly access.json').write_text(json.dumps(dict(checks=checks,passed=True,scope='Rod and pin straight insertion sweeps; 33 sampled carriage-half approach positions. Does not validate the full module assembly sequence.'),indent=2))
