import os
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];out=[]
for name,normal,heights in [('Locking bolt guide',[0,0,-1],[.5,1.7]),('Carriage body',[-1,0,0],[8.3,14.7]),('Module base',[0,1,0],[10.5,10.7])]:
 t=trimesh.load(Path(os.environ.get('PLANAR_OUTPUT',str(R/'adapted')))/(name+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 for z in heights:
  previous=s.slice(z-.2);current=s.slice(z);unsupported=current-previous.offset(.2)
  islands=[]
  for c in current.decompose():
   if (c^previous.offset(.2)).area()<.01 and c.area()>.1:
    a=np.concatenate(c.to_polygons());islands.append(dict(area_mm2=c.area(),bounds=[a.min(0).tolist(),a.max(0).tolist()]))
  out.append(dict(part=name,layer_sample_height=z,previous_sample_height=z-.2,unsupported_area_beyond_45_degree_step_mm2=unsupported.area(),unattached_islands=islands))
(R/'design-review/Layer onset checks.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2),flush=True)
