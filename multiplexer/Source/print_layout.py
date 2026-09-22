"""Package exactly the printed geometry used by the current two-piece viewer."""
from pathlib import Path
import json, hashlib
import numpy as np
import trimesh
from clean_print_mesh import clean
O=Path(__file__).resolve().parents[1];D=O
parts=json.loads((O/'printed-parts.json').read_text())
plate=[];manifest=[];x=16.;y=16.;row=0.;gap=12.
for p in parts:
 source=Path(p.get('source',O/(p['id']+'.stl')))
 t=clean(trimesh.load(source));assert t.is_watertight and len(t.split())==1,p['id']
 t.apply_transform(trimesh.transformations.rotation_matrix(p['print_rotation_angle'],p['print_rotation_axis']))
 t.apply_translation(-t.bounds[0]);size=t.extents
 if x+size[0]>240:x=16.;y+=row+gap;row=0.
 t.apply_translation([x,y,0]);assert np.all(t.bounds[0]>=-1e-6) and np.all(t.bounds[1,:2]<=256)
 bed=(np.abs(t.triangles[:,:,2])<1e-5).all(axis=1)
 assert t.area_faces[bed].sum()>1,p['id']
 manifest.append(dict(part=p['id'],source=source.name,sha256=hashlib.sha256(source.read_bytes()).hexdigest(),position_mm=[x,y,0],size_mm=size.tolist(),bed_contact_mm2=float(t.area_faces[bed].sum())))
 plate.append(t);x+=size[0]+gap;row=max(row,size[1])
for i,a in enumerate(plate):
 for b in plate[i+1:]:
  overlap=np.minimum(a.bounds[1,:2],b.bounds[1,:2])-np.maximum(a.bounds[0,:2],b.bounds[0,:2]);assert not np.all(overlap>0)
out=D/'Complete print layout.stl'
trimesh.util.concatenate(plate).export(out,file_type="stl_ascii")
for _ in range(2):
 clean(trimesh.load(out)).export(out,file_type="stl_ascii")
check=trimesh.load(out);components=check.split();assert check.is_watertight and len(components)==10
assert all(abs(c.bounds[0,2])<1e-5 for c in components)
(D/'Print layout manifest.json').write_text(json.dumps(dict(parts=manifest,solid_count=len(components),watertight=check.is_watertight,bounds_mm=check.bounds.tolist(),notes='Current aligned 16T multiplexer. LEGO hardware excluded. Orientations preserved; no generated supports or slicing included.'),indent=2))
print(out);print('10 closed solids; all on bed; separated footprints; bounds:',check.bounds.tolist())
