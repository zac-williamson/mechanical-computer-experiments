"""Audit every retained print, not just the newly designed cartridge supports."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh
from wall_print_geometry import qualify
R=Path(__file__).resolve().parents[1];O=R/'Functional register';OLD=R/'Wall register'
P=json.load(open(O/'parts.json'));V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);D=json.load(open(O/'Architecture.json'))
orient={}
for fn in ['Axle print-face audit.json','Assembly print orientations.json','Storage module print checks.json']:
 for e in json.load(open(OLD/fn)).get('parts',[]):
  ax=e.get('axis',e.get('print_up_axis'))
  if ax is not None:orient[e['part']]=('XYZ'.index(ax) if isinstance(ax,str) else ax,e['sign'])
rows=[]
for p in P:
 n=p['id']
 if p['kind']!='printed' or n in D['print_orientations']:continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']];m=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 if '_gate ' in n:ax,sign=0,(-1 if 'fork and roof' in n else 1)
 elif n.endswith('cheek carrier'):ax,sign=1,-1
 else:ax,sign=orient[n]
 t,q=qualify(m,ax,sign)
 row=dict(part=n,axis=ax,sign=sign,**q,unchanged_from_wall=True)
 if n.endswith('cheek carrier'):row['static_only_overhang_scope']='Unchanged fixture without running or axle-bearing surfaces; user accepted static overhangs.'
 if not q['print_geometry_pass']:
  bad=(t.face_normals[:,2]<-np.sqrt(.5)-1e-5)&(t.triangles_center[:,2]>.001)
  row['unsupported_regions_in_assembly']=m.triangles_center[bad].tolist()
 rows.append(row)
 if q['print_geometry_pass']:t.export(O/'Candidate print parts'/(n+'.stl'))
 else:(O/'Candidate print parts'/(n+'.stl')).unlink(missing_ok=True)
 print(n,q['print_geometry_pass'],round(q['unsupported_face_area_mm2'],3),len(q['layer_growth_failures']),flush=True)
(O/'Inherited manufacturing checks.json').write_text(json.dumps(dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts=rows,slicer_validated=False,physically_validated=False),indent=2))
