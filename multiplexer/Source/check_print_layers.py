from pathlib import Path
import trimesh,numpy as np,manifold3d as m,json
from shapely.geometry import Polygon
from shapely import union_all,points,distance
O=Path(__file__).resolve().parents[1]
def shape(cs):
 rings=cs.to_polygons();s=Polygon()
 for r in rings:
  if len(r)>2:s=s.symmetric_difference(Polygon(r))
 return s.buffer(0)
report=[]
for path in O.glob('* - print.stl'):
 name=path.stem[:-8]
 t=trimesh.load(O/(name+' - print.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(np.round(t.vertices,5)),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 prev=None;rows=[]
 for z in np.arange(.1,t.bounds[1,2],.2):
  cur=shape(s.slice(float(z)))
  if prev is not None and not cur.is_empty:
   unsupported=cur.difference(prev.buffer(.205))
   if unsupported.area>.015:
    coords=np.array(cur.boundary.segmentize(.1).coords) if cur.boundary.geom_type=='LineString' else np.concatenate([np.array(g.segmentize(.1).coords) for g in cur.boundary.geoms])
    maxreach=float(distance(points(coords),prev).max())
    rows.append(dict(z=round(float(z),3),beyond_45deg_area_mm2=round(unsupported.area,3),maximum_step_mm=round(maxreach,3),bounds=list(unsupported.bounds)))
  prev=cur
 report.append(dict(part=name,layer_height=.2,flagged_layers=rows));print(name,len(rows),'max',max((r['maximum_step_mm'] for r in rows),default=0),flush=True)
print(json.dumps(report,indent=2))
