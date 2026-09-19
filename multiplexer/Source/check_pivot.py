"""Check current exported meshes; run with numpy, trimesh and manifold3d installed.
This is a pivot/frame regression check, not the complete contact/tolerance audit.
"""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1];d=json.loads((O/'Design parameters.json').read_text());pv=np.array(d['pivot'])
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def cyl(r,a,b):return m.Manifold.cylinder(b-a,r,circular_segments=128).translate([*pv,a])
parts={x.stem:solid(trimesh.load(x)) for x in O.glob('*.stl') if ' - print' not in x.stem and 'layout' not in x.stem}
checks=[]
for q in np.linspace(-4.6,4.575,185):
 for name in ['Left carriage half','Right carriage half']:
  car=parts[name].translate([float(q),0,0])
  for label,other in [('axle clearance',cyl(3.,10.4,58.4)),('bushing clearance',cyl(5.30,32.2,48.2)),('front bridge',parts['Front actuator bridge']),('rear bridge',parts['Rear bridge and band anchor'])]:
   vol=(car^other).volume()
   if vol>.001:checks.append([name,float(q),label,vol])
for beta in np.arange(-28,28.01,.25):
 lev=parts['Direct lever and band cleat'].translate([-pv[0],-pv[1],0]).rotate([0,0,float(beta)]).translate([*pv,0])
 for label,other in [('front bridge',parts['Front actuator bridge']),('rear bridge',parts['Rear bridge and band anchor']),('bushes',cyl(3.95,32.2,48.2))]:
  vol=(lev^other).volume()
  if vol>.001:checks.append(['lever',float(beta),label,vol])
assert not checks,checks[:20]
print('Exported pivot/frame regression passed: 185 carriage positions, 225 lever angles; both bridges included.')
