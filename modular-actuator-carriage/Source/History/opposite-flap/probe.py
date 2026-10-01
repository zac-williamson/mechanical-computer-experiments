from pathlib import Path
import trimesh,numpy as np,manifold3d as m
R=Path(__file__).resolve().parents[1]
for name in ['Carriage body','Carriage bearing end']:
 t=trimesh.load_mesh(R/'neck-candidate'/(name+'.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 for ya,yb in [(12.701,18.21),(1.54,18.21)]:
  cut=m.Manifold.cube([18.198,yb-ya,7.4001]).translate([-9.099,ya,3.9]);q=s^cut
  print(name,ya,[(c.volume(),c.bounding_box()) for c in q.decompose()],flush=True)
