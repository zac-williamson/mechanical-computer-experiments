from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1]
def load(n):
 t=trimesh.load(R/'simplified-carriage-r1'/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
for sign,n in [(-1,'Carriage bearing end'),(1,'Carriage body')]:
 s=load(n)
 for xa,y0,y1 in [(12,14.4,17.5),(12,14.4,20),(9.1,14.4,17.5),(12,10,14.4)]:
  a,b=(-16.25,-xa) if sign<0 else(xa,16.25);p=box([a,y0,19.5],[b,y1,30]);added=p-s
  print(n,xa,y0,y1,'added',added.volume(),'bounds',added.bounding_box(),flush=True)
