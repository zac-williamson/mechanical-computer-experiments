"""Export the actual frames rear-face down and audit unsupported mesh faces."""
from pathlib import Path
import numpy as np,trimesh,json,hashlib
R=Path(__file__).resolve().parents[1];O=R/'Wall register';D=O/'Print oriented frames';D.mkdir(exist_ok=True)
for f in D.glob('*.stl'):f.unlink()
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);out=[]
for p in P:
 if 'coordinated chassis' not in p['id']:continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']];mesh=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 T=trimesh.geometry.align_vectors([0,-1,0],[0,0,1]);mesh.apply_transform(T);shift=-mesh.bounds[0];mesh.apply_translation(shift)
 bad=(mesh.face_normals[:,2]<-np.sqrt(.5)-1e-5)&(mesh.triangles_center[:,2]>.001)
 area=float(mesh.area_faces[bad].sum());bed=(abs(mesh.triangles[:,:,2]).max(1)<.001)
 filename={'bit coordinated chassis 0':'Bit frame left.stl','bit coordinated chassis 1':'Bit frame right.stl','control coordinated chassis 0':'Control frame.stl'}[p['id']]
 path=D/filename;mesh.export(path)
 out.append(dict(part=p['id'],file=str(path.relative_to(O)),bounds_mm=mesh.extents.tolist(),flat_bed_contact_mm2=float(mesh.area_faces[bed].sum()),unsupported_area_over_45_degrees_mm2=area,unsupported_face_bounds=[mesh.triangles[bad].reshape(-1,3).min(0).tolist(),mesh.triangles[bad].reshape(-1,3).max(0).tolist()] if bad.any() else None,connected_solids=len(mesh.split()),watertight=bool(mesh.is_watertight),assembly_to_print_rotation=T.tolist(),print_translation_mm=shift.tolist(),pass_check=area<.01 and mesh.is_watertight and len(mesh.split())==1))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope='Frames only: rear face on bed, vertical mounting holes, 45-degree roofs over horizontal frame joining pins. No bridge exemption.',parts=out,frame_print_geometry_pass=all(a['pass_check'] for a in out),limitations=['A geometric overhang screen, not a printer/material calibration.','Detached working fixtures and bearing walls are separate parts; this report does not qualify their print orientations.'])
(O/'Frame print checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
