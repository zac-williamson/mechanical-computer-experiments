from pathlib import Path
import trimesh,numpy as np,manifold3d as m
p=Path(__file__).resolve().parents[1]/'reinforced-hooks-candidate/Locking bolt guide.stl';t=trimesh.load_mesh(p);g=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
for h in [43,43.16]:
 a=g.slice(h)-g.slice(h-.1599).offset(.1601, m.JoinType.Round, circular_segments=128)
 print(h,a.area(),a.to_polygons(),flush=True)
