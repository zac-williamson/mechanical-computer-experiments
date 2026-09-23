"""Reproduce the native surface contact screen; not a loaded engagement model."""
from pathlib import Path
import json,numpy as np,trimesh
O=Path(__file__).resolve().parents[1]/'Planar register'
v=np.load(O/'hardware.npz')['vertices'];hs={h['id']:h for h in json.loads((O/'hardware.json').read_text())}
def mesh(n):
 h=hs[n];a=v[h['offset']//3:h['offset']//3+h['vertices']];return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False)
a=trimesh.collision.CollisionManager();b=trimesh.collision.CollisionManager();a.add_object('ring',mesh('Memory — L099'));b.add_object('gear',mesh('Memory — L072'));rows=[]
for x in np.arange(.8,3.351,.025):
 hits=[]
 for d in range(360):
  t=trimesh.transformations.rotation_matrix(np.radians(d),[1,0,0],[0,10.2,0]);t[0,3]=float(x);a.set_transform('ring',t)
  if a.in_collision_other(b):hits.append(d)
 rows.append(dict(ring_shift_mm=float(x),contact_phases_deg=hits))
report=dict(method='Native triangle surface intersection, 1 degree relative phase and 0.025 mm axial increments; potential flank contact, not a loaded clutch or chamfer simulation. Empty phase samples do not certify an interference-free solid.',first_sample_with_contact_mm=next(r['ring_shift_mm'] for r in rows if r['contact_phases_deg']),samples=rows)
(O/'Clutch phase contact scan.json').write_text(json.dumps(report,indent=2))
