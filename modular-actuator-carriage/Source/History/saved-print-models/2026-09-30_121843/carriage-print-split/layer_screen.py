from pathlib import Path
import os,json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=Path(os.environ.get('PLANAR_OUTPUT',str(R/'split-candidate')));rows=[]
for name,rot in [('Carriage body',[0,-90,0]),('Carriage bearing end',[0,90,0])]:
 t=trimesh.load(O/(name+'.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64))).rotate(rot);b=s.bounding_box();s=s.translate([-b[0],-b[1],-b[2]]);layers=[]
 for h in np.arange(.3,s.bounding_box()[5],.2):
  old=s.slice(h-.2);now=s.slice(h);extra=now-old.offset(.21);islands=[c.area() for c in now.decompose() if c.area()>.1 and (c^old.offset(.21)).area()<.01]
  if extra.area()>.5 or islands:layers.append(dict(height=round(float(h),2),unsupported_area_mm2=extra.area(),detached_islands_mm2=islands))
 rows.append(dict(part=name,layers=layers))
(O/'Carriage layer screening.json').write_text(json.dumps(rows,indent=2));assert not any(l['detached_islands_mm2'] for r in rows for l in r['layers'])
print('Both carriage pieces have no detached islands in the 0.2 mm layer screen. Short bridges remain; see report.',flush=True)
