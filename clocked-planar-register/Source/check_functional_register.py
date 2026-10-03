"""Manufacturing checks for the candidate. No STL publishing on failed new parts."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
from wall_print_geometry import qualify
from wall_pose import vertices as posed
R=Path(__file__).resolve().parents[1];O=R/'Functional register'
P=json.load(open(O/'parts.json'));V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);D=json.load(open(O/'Architecture.json'))
L={p['id']:p for p in P}
def vv(p):return V[p['offset']//3:p['offset']//3+p['vertices']]
def mesh(a):return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
def solid(a):
 t=mesh(a);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
prints=[];paths=O/'Candidate print parts';paths.mkdir(exist_ok=True)
for path in paths.glob('*.stl'):
 if path.stem not in L:path.unlink()
for name,(axis,sign) in D['print_orientations'].items():
 p=L[name];t,q=qualify(mesh(vv(p)),axis,sign)
 if p.get('inherited_cam') and not q['print_geometry_pass']:
  # Diagnose orientations rather than burying support-dependent working faces.
  options=[]
  for ax in range(3):
   for si in [-1,1]:
    tt,qq=qualify(mesh(vv(p)),ax,si);options.append((qq['unsupported_face_area_mm2'],ax,si,tt,qq))
  area,axis,sign,t,q=min(options,key=lambda a:a[0])
 row=dict(part=name,axis=axis,sign=sign,**q);prints.append(row)
 if q['print_geometry_pass']:t.export(paths/(name+'.stl'))
 else:(paths/(name+'.stl')).unlink(missing_ok=True)
 print('PRINT',name,q['print_geometry_pass'],round(q['unsupported_face_area_mm2'],3),len(q['layer_growth_failures']),flush=True)
(O/'Candidate manufacturing checks.json').write_text(json.dumps(dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts=prints,new_parts_print_pass=all(q['print_geometry_pass'] for q in prints),slicer_validated=False,physically_validated=False),indent=2))
