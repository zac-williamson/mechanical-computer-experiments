from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parent;O=R.parent/'adapted'
def load(path):
 t=trimesh.load(path);return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,c[0],c[1]])
old=load(R/'before/Carriage body.stl');new=load(O/'Carriage body.stl');removed=old-new
checks=[]
for label,region in [('Original clutch-contact material',cy(7.4,-8,7.8,[10.2,0])),('Original lever working profile',box([-30,30,11.6],[30,37.49,20]))]:
 loss=(removed^region).volume();checks.append(dict(check=label,removed_mm3=loss,passed=loss<.02))
for n,a,b in [('Carriage body',-16.24,-9.11),('Carriage bearing end',9.11,16.24)]:
 intrusion=(load(O/(n+'.stl'))^cy(2.83,a,b,[10.2,16])).volume();checks.append(dict(check=n+' axle bore remains clear',intrusion_mm3=intrusion,passed=intrusion<.02))
(O/'Carriage interface checks.json').write_text(json.dumps(checks,indent=2));print(json.dumps(checks,indent=2),flush=True)
