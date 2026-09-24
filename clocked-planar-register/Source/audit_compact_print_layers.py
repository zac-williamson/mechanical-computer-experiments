"""Layer connectivity of proposed print orientations at 0.2 mm layers.

A newly appearing island with no preceding-layer contact cannot print without
support. Passing this check does not validate bridges, cooling or hole finish.
"""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1]/'Compact layout';ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3);orient=json.loads((R/'Print orientation screening.json').read_text());lookup={p['id']:p for p in ps};rows=[]
for part in orient['parts']:
 p=lookup[part['part']];a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);s=m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
 choice=part['best_screened_orientation'];up=np.eye(3)['XYZ'.index(choice['up_axis'])]*choice['sign'];matrix=trimesh.geometry.align_vectors(up,[0,0,1]);s=s.transform(matrix[:3]);b=s.bounding_box();s=s.translate([0,0,-b[2]]);height=b[5]-b[2]
 prev=None;bad=[];unsupported=0.
 for z in np.arange(.1,height,.2):
  cur=s.slice(float(z))
  if prev is not None:
   for c in cur.decompose():
    if c.area()>.01 and (c^prev.offset(.02)).area()<.001:bad.append(dict(height_mm=float(z),island_area_mm2=c.area(),bounds=c.bounds()))
   unsupported=max(unsupported,(cur-prev.offset(.2*np.tan(np.deg2rad(55)))).area())
  prev=cur
 rows.append(dict(part=p['id'],orientation=choice,new_islands=bad,maximum_area_requiring_bridge_or_support_mm2=unsupported,no_new_islands=not bad))
 print(p['id'],len(bad),round(unsupported,2),flush=True)
r=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),layer_height_mm=.2,parts=rows,all_layers_connected=all(x['no_new_islands'] for x in rows),mechanically_qualified=False)
(R/'Print layer screening.json').write_text(json.dumps(r,indent=2))
