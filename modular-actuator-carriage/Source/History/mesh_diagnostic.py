from pathlib import Path
import trimesh,numpy as np,sys
R=Path(__file__).resolve().parent
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
from clean_print_mesh import clean
for n in ['Carriage body','Carriage bearing end']:
 t=trimesh.load(R/'adapted'/(n+'.stl'));print(n,len(t.faces),t.volume,t.is_watertight,flush=True)
 for digits in [6,5,4,3]:
  q=t.copy();q.vertices=np.round(q.vertices,digits);q.merge_vertices();q.update_faces(q.unique_faces());q.update_faces(q.nondegenerate_faces(height=1e-7));q.remove_unreferenced_vertices();q=clean(q)
  print(digits,len(q.faces),q.volume,q.is_watertight,flush=True)
