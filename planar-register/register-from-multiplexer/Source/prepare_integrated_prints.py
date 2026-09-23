from pathlib import Path
import json,hashlib,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R.parent/'work/integrated-cam-development';out=O/'Prototype print parts';out.mkdir(exist_ok=True);rows=[]
for p in json.loads((O/'printed-parts.json').read_text()):
 t=trimesh.load(O/(p['id']+'.stl'));assert t.is_watertight and len(t.split())==1
 if p['motion']=='carriage':normal=[-1,0,0]
 elif p['id'].endswith('Right side frame'):normal=[-1,0,0]
 elif p['id']=='Direct lock bolt':normal=[0,-1,0]
 elif p['id']=='Unified rear backbone':normal=[0,1,0]
 elif 'print_rotation_axis' in p:
  M=trimesh.transformations.rotation_matrix(p['print_rotation_angle'],p['print_rotation_axis']);normal=None
 else:normal=[0,-1,0]
 if normal is not None:M=trimesh.geometry.align_vectors(normal,[0,0,-1])
 t.apply_transform(M);t.apply_translation(-t.bounds[0]);t.export(out/(p['id']+'.stl'),file_type='stl_ascii');r=trimesh.load(out/(p['id']+'.stl'));assert r.is_watertight and len(r.split())==1
 bed=float(r.area_faces[np.all(abs(r.triangles[:,:,2])<1e-5,axis=1)].sum())
 rows.append(dict(part=p['id'],size_mm=r.extents.tolist(),flat_bed_area_mm2=bed,watertight=True,bearing_bed_preserved=p['motion']=='carriage',sha256=hashlib.sha256((O/(p['id']+'.stl')).read_bytes()).hexdigest()))
(O/'Print orientations.json').write_text(json.dumps(dict(parts=rows,limits='Geometry and bed orientations checked. No slicer/toolpath validation. Base and detachable guide have dedicated 0.2 mm geometric printability checks. The guide has one 1.2 mm band-groove bridge. Small fit test required before full print.'),indent=2));print('Prepared',len(rows),'watertight print-oriented parts')
