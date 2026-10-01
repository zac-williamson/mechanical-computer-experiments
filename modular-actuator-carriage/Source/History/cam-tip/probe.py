from pathlib import Path
import numpy as np,trimesh,manifold3d as m
r=Path(__file__).resolve().parents[1]/'neck-candidate'
t=trimesh.load_mesh(r/'Lock control rod.stl');s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
for y in [14.3,17,19.5]:
 sec=s.rotate([90,0,0]).slice(y)
 print(y,[p.tolist() for p in sec.to_polygons()],flush=True)
