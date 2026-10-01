from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1]/'neck-candidate'
for name in ['Carriage body','Carriage bearing end']:
 t=trimesh.load_mesh(R/(name+' print.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 out=[]
 for h in np.arange(.2,t.extents[2],.2):
  unsupported=s.slice(h)-s.slice(h-.2).offset(.202,m.JoinType.Round,circular_segments=128)
  if unsupported.area()>.1:out.append([round(float(h),2),round(unsupported.area(),3),unsupported.bounds()])
 print(name,json.dumps(out),flush=True)
