"""Remove float32-degenerate exported triangles; require round-trip valid solids."""
from pathlib import Path
import json,numpy as np,trimesh
O=Path(__file__).resolve().parents[1]/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());results=[]
for p in meta:
 path=O/(p['id']+'.stl');t=trimesh.load(path);before=len(t.faces)
 t.update_faces(t.nondegenerate_faces(height=1e-5));t.update_faces(t.unique_faces());t.remove_unreferenced_vertices();t.merge_vertices(digits_vertex=5);t.update_faces(t.nondegenerate_faces(height=1e-5));t.update_faces(t.unique_faces());t.remove_unreferenced_vertices()
 if t.is_watertight and len(t.split())==1:
  t.export(path);r=trimesh.load(path);ok=r.is_watertight and len(r.split())==1
 else:ok=False
 results.append(dict(part=p['id'],removed_triangles=before-len(t.faces),valid_roundtrip=bool(ok)))
 p['watertight']=bool(ok)
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2));(O/'Export repair.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
