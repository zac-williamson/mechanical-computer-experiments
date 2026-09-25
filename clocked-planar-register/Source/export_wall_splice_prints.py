"""Export the two friction-pin splice bridges front-face down on the bed."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Wall register';D=O/'Print oriented rod splices';D.mkdir(exist_ok=True)
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);out=[]
for p in P:
 if not p['id'].endswith('pinned rod splice bridge'):continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 T=trimesh.geometry.align_vectors([0,1,0],[0,0,1]);t.apply_transform(T);shift=-t.bounds[0];t.apply_translation(shift)
 bad=(t.face_normals[:,2]<-np.sqrt(.5)-1e-5)&(t.triangles_center[:,2]>.001);area=float(t.area_faces[bad].sum())
 path=D/(p['id']+'.stl');t.export(path)
 out.append(dict(part=p['id'],file=str(path.relative_to(O)),assembly_to_print_rotation=T.tolist(),print_translation_mm=shift.tolist(),unsupported_area_mm2=area,pass_check=area<.01 and t.is_watertight and len(t.split())==1))
assert len(out)==2
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts=out,splice_print_pass=all(p['pass_check'] for p in out),scope='Pin-hole axes normal to bed; no unsupported faces beyond 45 degrees, no bridge exemption.')
(O/'Rod splice print checks.json').write_text(json.dumps(report,indent=2));print(report);assert report['splice_print_pass']
