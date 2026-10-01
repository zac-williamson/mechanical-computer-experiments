from pathlib import Path
import json,math
import numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'lock-guidance-candidate'
def load(n):
 t=trimesh.load_mesh(O/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
b=load('Locking bolt').translate([0,0,2.6]);end=load('Carriage bearing end');rows=[]
for q in [-3.75,3.75]:
 for axis in [[1,0,0],[0,1,0]]:
  for angle in [-1.5,1.5]:
   M=trimesh.transformations.rotation_matrix(math.radians(angle),axis,[-5.05,27.6,50.2]);s=b.transform(M[:3])
   for dx,dy in [(-.3,-.2),(-.3,.2),(.3,-.2),(.3,.2)]:
    vol=(s^end.translate([q+dx,dy,0])).volume();rows.append(dict(q=q,axis=axis,angle=angle,dx=dx,dy=dy,overlap=vol));assert vol<.02,rows[-1]
(O/'Tilted entry checks.json').write_text(json.dumps(dict(passed=True,checks=rows,limits='Entry geometry only; spring-driven self-centering and friction require physical testing.'),indent=2));print('Tilted entry passed',flush=True)
