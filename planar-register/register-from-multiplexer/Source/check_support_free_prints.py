"""Geometric support check in the exported print orientation; no slicer invocation."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';rows=[]
def shape(section):
 result=Polygon()
 for ring in section.to_polygons():
  if len(ring)>=3:result=result.symmetric_difference(Polygon(ring))
 assert abs(result.area-section.area())<1e-5
 return result

for name in ['Unified rear backbone','Detachable bolt guide']:
 t=trimesh.load(O/'Prototype print parts'/(name+'.stl'));assert t.is_watertight
 bad=(t.face_normals[:,2]<-np.cos(np.pi/4)-1e-5)&(t.triangles[:,:,2].max(1)>1e-4);area=float(t.area_faces[bad].sum())
 bridge=box(16.4,18.4,21.2,19.6) if name=='Detachable bolt guide' else Polygon()
 if name=='Detachable bolt guide':
  triangles=t.triangles[bad];assert np.all(triangles[:,:,1]>=18.4-1e-5) and np.all(triangles[:,:,1]<=19.6+1e-5)
  assert np.all(triangles[:,:,2]>=2.1-1e-5) and np.all(triangles[:,:,2]<=2.81)
 else:assert area<.001,(name,area)
 s=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)));last=shape(s.slice(.001));lastz=.001;worst=0;islands=0;layers=0
 step=.2
 for z in np.arange(step,t.bounds[1,2],step):
  p=shape(s.slice(float(z)));support=last.buffer(float(z-lastz)+.015,quad_segs=16);excess=p.difference(support);
  if 2.0<z<2.82 and not bridge.is_empty:
   assert last.buffer(1e-5).covers(box(16.4,18.3,21.2,18.4)) and last.buffer(1e-5).covers(box(16.4,19.6,21.2,19.7))
   excess=excess.difference(bridge)
  if excess.area>max(worst,.02):print(name,'layer',z,'excess',excess.area,'bounds',excess.bounds,flush=True)
  worst=max(worst,float(excess.area));components=list(p.geoms) if p.geom_type=='MultiPolygon' else [p]
  islands+=sum(not c.is_empty and not c.intersects(support) for c in components);last=p;lastz=z;layers+=1
 rows.append(dict(part=name,downward_faces_steeper_than_45_degrees_area_mm2=area,explicit_supported_bridge_span_mm=1.2 if name=='Detachable bolt guide' else 0,layer_height_mm=step,layers=layers,unsupported_islands=islands,maximum_layer_area_beyond_45_degree_support_mm2=worst,bed_contact_mm2=float(t.area_faces[np.all(abs(t.triangles[:,:,2])<1e-5,axis=1)].sum())))
 assert islands==0 and worst<.02,rows[-1]
r=dict(parts=rows,method='Printed mesh face normals plus cross sections every 0.2 mm. Base: 0.2 mm layers with 45-degree allowance plus 0.015 mm numerical tolerance. Guide: the same test, with one explicit 1.2 mm bridge across the elastic-band groove; solid support is verified at both ends in the preceding layer. No other steep faces or unsupported areas allowed. Both require zero isolated layer components.',limits='Geometric printability check, not a sliced toolpath or physical print guarantee. Guide includes a 1.2 mm bridge over its band groove. No Bambu Studio or other slicer was used.');(O/'Support-free print checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
