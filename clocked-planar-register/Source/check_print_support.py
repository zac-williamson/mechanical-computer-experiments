"""Conservative geometric print screening; bridges remain reported, no slicer."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from shapely.geometry import Polygon
R=Path(__file__).resolve().parents[1];O=R/'Assembly development'
def shape(cs):
 p=Polygon()
 for a in cs.to_polygons():
  if len(a)>=3:p=p.symmetric_difference(Polygon(a))
 return p
rows=[]
for p in json.loads((O/'parts.json').read_text()):
 if p['kind']!='printed':continue
 n=p['id'];normal=[0,-1,0]
 if n.startswith('Base ') or n=='Clock rail drive post':normal=[0,1,0]
 elif p['motion'] in ['carriage','fork']:
  normal=[1,0,0] if 'Right carriage' in n else [-1,0,0]
 elif n.startswith('Rail guide '):normal=[-1,0,0]
 elif 'bearing' in n.lower() and 'cheek' not in n.lower():normal=[-1,0,0]
 elif 'side frame' in n:normal=[-1,0,0]
 t=trimesh.load(O/(n+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0])
 s=m.Manifold(m.Mesh64(t.vertices,t.faces.astype(np.uint64)));last=shape(s.slice(.001));lastz=.001;worst=0.;islands=0;worstz=0
 for z in np.arange(.2,t.bounds[1,2],.2):
  current=shape(s.slice(float(z)));support=last.buffer(float(z-lastz)+.015);excess=current.difference(support)
  if excess.area>worst:worst=float(excess.area);worstz=float(z)
  polys=list(current.geoms) if current.geom_type=='MultiPolygon' else [current]
  islands+=sum(not q.is_empty and not q.intersects(support) for q in polys)
  last=current;lastz=z
 row=dict(part=n,bed_outward_normal=normal,size_mm=t.extents.tolist(),maximum_unsupported_layer_area_mm2=worst,worst_layer_mm=worstz,unsupported_islands=islands,passes_strict_no_bridge_screen=worst<.02 and islands==0)
 rows.append(row);print(n,round(worst,3),islands,flush=True)
(R/'Print support screening.json').write_text(json.dumps(dict(scope='0.2 mm layers and 45-degree support envelope; no bridge exemptions, toolpaths or actual print qualification',parts=rows),indent=2))
