from pathlib import Path
import trimesh,numpy as np,manifold3d as m,json
R=Path(__file__).resolve().parent/'adapted'
def solid(n):
 t=trimesh.load(R/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
result=[]
for n,a,b in [('Carriage body',.2,7.7),('Carriage bearing end',8.7,15.5)]:
 s=solid(n)
 for y,z in [(24.7,.5),(35.4,32)]:
  ring=cy(3.5,a,b,0,[0,y,z])-cy(2.56,a-.01,b+.01,0,[0,y,z]);missing=(ring-s).volume()
  result.append(dict(part=n,pin=[y,z],missing_surround_mm3=missing,pass_check=missing<.01))
(R/'Joint checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
