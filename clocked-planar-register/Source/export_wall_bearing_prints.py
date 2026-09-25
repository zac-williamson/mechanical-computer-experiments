"""Orient removable bearing walls with their axle bores normal to the bed."""
from pathlib import Path
import numpy as np,trimesh,json,hashlib
R=Path(__file__).resolve().parents[1];O=R/'Wall register';D=O/'Print oriented bearing walls';D.mkdir(exist_ok=True)
for old in D.glob('*.stl'):old.unlink()
ps={p['id']:p for p in json.loads((O/'parts.json').read_text())};v=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
s=json.loads((O/'Bearing schedule.json').read_text());out=[]
face_audit=json.loads((O/'Axle print-face audit.json').read_text())
assert face_audit['geometry_sha256']==hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
faces={p['part']:p for p in face_audit['parts']}
for entry in s['removable_walls']:
 p=ps[entry['part']];a=v[p['offset']//3:p['offset']//3+p['vertices']]
 mesh=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 normal=np.eye(3)['XYZ'.index(entry['bearing_axis'])]
 rot=trimesh.geometry.align_vectors(normal,[0,0,1]);mesh.apply_transform(rot);shift=-mesh.bounds[0];mesh.apply_translation(shift)
 path=D/(entry['part']+'.stl');mesh.export(path)
 out.append(dict(part=entry['part'],file=str(path.relative_to(O)),bed_normal_alignment=float((rot[:3,:3]@normal)[2]),bed_min_z_mm=float(mesh.bounds[0,2]),assembly_to_print_rotation=rot.tolist(),print_translation_mm=shift.tolist(),mounting_pins=len(entry['pins']),bearing_rings_on_bed=faces[entry['part']]['axle_bed_face_pass'],bearing_ring_count=len(faces[entry['part']]['holes'])))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts=out,orientation_pass=all(x['bed_normal_alignment']>.999999 and abs(x['bed_min_z_mm'])<1e-6 and x['bearing_rings_on_bed'] for x in out),limitations=['Hole axes are vertical and all scheduled bearing rings meet the bed. Slicer review of stepped faces, mounting-pad overhangs and horizontal pin holes is still required.','This export is not a material, fit, strength or print-support qualification.'])
(O/'Bearing print orientations.json').write_text(json.dumps(report,indent=2));print('Oriented',len(out),'walls')
