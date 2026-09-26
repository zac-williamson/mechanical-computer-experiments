"""Export new detachable pieces with their specified broad faces on the bed."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh
from wall_extra_connections import extra_connections
O=Path(__file__).resolve().parents[1]/'Wall register';D=O/'Print oriented assembly parts';D.mkdir(exist_ok=True)
for f in D.glob('*.stl'):f.unlink()
P={p['id']:p for p in json.loads((O/'parts.json').read_text())};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
entries=extra_connections(O)+[dict(part='Control WRITE direct rod and pickup',print_axis=1,print_up_sign=1)];out=[]
for e in entries:
 p=P[e['part']];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 axis=e.get('print_axis',1);sign=e.get('print_up_sign',1);normal=np.eye(3)[axis]*sign;rot=trimesh.geometry.align_vectors(normal,[0,0,1]);t.apply_transform(rot);translation=-t.bounds[0];t.apply_translation(translation)
 bad=(t.face_normals[:,2]<-.70712)&(t.triangles_center[:,2]>.001);path=D/(p['id']+'.stl');t.export(path)
 out.append(dict(part=p['id'],file=str(path.relative_to(O)),print_up_axis='XYZ'[axis],sign=sign,assembly_to_print_rotation=rot.tolist(),print_translation_mm=translation.tolist(),watertight=bool(t.is_watertight),connected_solids=len(t.split()),unsupported_area_mm2=float(t.area_faces[bad].sum())))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts=out,scope=__doc__,export_pass=all(p['watertight'] and p['connected_solids']==1 for p in out),limitations=['Other static non-running overhangs are accepted by the user; this report measures rather than certifies support-free printing. Axle-ring bed contact is checked separately.'])
(O/'Assembly print orientations.json').write_text(json.dumps(report,indent=2));print('Exported',len(out),'assembly pieces')
