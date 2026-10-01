from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
R=Path(__file__).resolve().parents[1]
t=trimesh.load(R/'adapted/Carriage body.stl');t.apply_transform(trimesh.geometry.align_vectors([-1,0,0],[0,0,-1]));t.apply_translation(-t.bounds[0]);s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def poly(cs):
 result=Polygon()
 for p in cs.to_polygons():result=result.symmetric_difference(Polygon(p))
 return result
out=[]
for z in [8.25,14.65]:
 prior=poly(s.slice(z-.01));after=poly(s.slice(z+.01));new=after-prior
 pts=[]
 for x in np.arange(new.bounds[0],new.bounds[2],.2):
  for y in np.arange(new.bounds[1],new.bounds[3],.2):
   p=Point(x,y)
   if new.contains(p):pts.append((p.distance(prior),x,y))
 out.append(dict(height=z,new_area=new.area,max_distance_to_existing_material=max(pts) if pts else None,polygons=[p.tolist() for p in s.slice(z+.01).to_polygons()]))
print(json.dumps(out,indent=2),flush=True);(R/'print-revision/step-analysis.json').write_text(json.dumps(out,indent=2))
