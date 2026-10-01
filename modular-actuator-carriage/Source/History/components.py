from pathlib import Path
import trimesh,manifold3d as m,numpy as np
P=Path('/Users/zac/Documents/ChatGPT/lego designs 2_/work/planar-module-restart/adapted')
for n in ['Carriage body','Carriage bearing end']:
 t=trimesh.load(P/(n+'.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 print(n,flush=True)
 for c in s.decompose():print(c.volume(),c.bounding_box(),flush=True)
