"""Read-only geometry audit of released prototype files; writes audit evidence only."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());hw=json.loads((O/'hardware.json').read_text());out=[]
for p in meta:
 row={'part':p['id']}
 for kind,path in [('assembly',O/(p['id']+'.stl')),('print',O/'Prototype print parts'/(p['id']+'.stl'))]:
  t=trimesh.load(path);edges=np.bincount(t.edges_unique_inverse);row[kind]=dict(watertight=bool(t.is_watertight),winding_consistent=bool(t.is_winding_consistent),boundary_edges=int(sum(edges==1)),nonmanifold_edges=int(sum(edges>2)),components_including_open=len(t.split(only_watertight=False)),size_mm=t.extents.tolist(),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
 out.append(row)
report=dict(print_meshes=out,hardware_count=len(hw),explicit_catalog_id_count=sum('lego_part' in h for h in hw),native_without_catalog_id=[h['id'] for h in hw if 'lego_part' not in h],torque_screen=dict(axle_torque_Nm=.1,tangential_force_8T_pitch_radius_4mm_N=.1/.004,tangential_force_16T_pitch_radius_8mm_N=.1/.008,ideal_blocked_worm_axial_force_assuming_pi_mm_lead_N=2*np.pi*.1/(np.pi/1000),note='Ideal lossless force illustrates missing load case; not a measured force or prediction of a particular failure torque.'),scope='STLs reloaded from disk, not in-memory source manifold. No physical load or print test.')
(O/'Criteria audit evidence.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='print_meshes'},indent=2));print('Mesh failures',[(r['part'],r['print']) for r in out if not r['print']['watertight']])
