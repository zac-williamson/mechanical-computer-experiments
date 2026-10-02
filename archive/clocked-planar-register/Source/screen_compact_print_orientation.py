"""Downward-face screening of all printed parts in six bed orientations.

Measures unsupported-looking face area above the first layer, not slicer
support requirements. Bridges, layer connectivity, holes and extrusion width
require separate checks; zero face area is not a printability certificate.
"""
from pathlib import Path
import json,hashlib,math
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1]/'Compact layout';ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3);rows=[]
for p in ps:
 if p['kind']!='printed':continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);options=[]
 for axis in range(3):
  for sign in [-1,1]:
   heights=t.triangles[:,:,axis]*sign;heights-=heights.min();bad=(t.face_normals[:,axis]*sign < -math.sin(math.radians(55))) & (heights.mean(1)>.25)
   bed=(heights.max(1)<.02)
   options.append(dict(up_axis='XYZ'[axis],sign=sign,height_mm=float(heights.max()),steep_downward_area_mm2=float(t.area_faces[bad].sum()),bed_contact_area_mm2=float(t.area_faces[bed].sum())))
 # Preserve the original bearing-flat intent where the main working bore is X.
 required='X' if any(k in p['id'] for k in ['Carriage fork and roof','Right carriage bearing support','Bearing support']) else None
 if p['id']=='Clock rear pivot bearing bridge':required='Y'
 choices=[x for x in options if x['up_axis']==required] if required else options
 if p['id'].startswith(('master_gate ','slave_gate ')):choices=[x for x in choices if x['up_axis']=='X' and x['sign']==1]
 if p['id']=='Clock rear pivot bearing bridge':choices=[x for x in choices if x['sign']==1]
 # A low downward-face area can still start a disconnected feature in air.
 # Test layer connectivity before ranking the orientations. This is still
 # not a bridge/cooling qualification and never labels an island-free part
 # support-free solely on that basis.
 solid=m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
 for option in choices:
  up=np.eye(3)['XYZ'.index(option['up_axis'])]*option['sign']
  s=solid.transform(trimesh.geometry.align_vectors(up,[0,0,1])[:3])
  bounds=s.bounding_box();s=s.translate([0,0,-bounds[2]])
  previous=None;islands=0
  for height in np.arange(.1,bounds[5]-bounds[2],.2):
   layer=s.slice(float(height))
   if previous is not None:
    islands+=sum(c.area()>.01 and (c^previous.offset(.02)).area()<.001 for c in layer.decompose())
   previous=layer
  option['new_layer_islands']=islands
 best=min(choices,key=lambda x:(x['new_layer_islands'],x['steep_downward_area_mm2'],-x['bed_contact_area_mm2']))
 rows.append(dict(part=p['id'],watertight=t.is_watertight,connected_bodies=len(t.split()),volume_mm3=t.volume,required_bearing_axis=required,best_screened_orientation=best,orientations=options))
r=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),parts=rows,mechanically_qualified=False)
(R/'Print orientation screening.json').write_text(json.dumps(r,indent=2));print(json.dumps([dict(part=x['part'],**x['best_screened_orientation']) for x in rows],indent=2))
