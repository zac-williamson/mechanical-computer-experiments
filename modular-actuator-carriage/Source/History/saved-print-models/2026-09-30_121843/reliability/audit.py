from pathlib import Path
import json,os,shutil,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'adapted';O=R/'reliability-candidate';O.mkdir(exist_ok=True)
for p in A.iterdir():
 if p.is_file():shutil.copy2(p,O/p.name)
os.environ['PLANAR_OUTPUT']=str(O)
s=(R/'design-review/print_audit.py').read_text().replace("R/'design-review/Print geometry audit.json'","O/'Current print audit.json'")
exec(compile(s,'current print audit','exec'),{'__name__':'__main__','__file__':str(R/'design-review/print_audit.py')})
D=json.loads((O/'Model.json').read_text());normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]};rows=[]
for p in D['parts']:
 if p['kind']!='printed':continue
 t=trimesh.load(O/(p['name']+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed']],[0,0,-1]));t.apply_translation(-t.bounds[0]);s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)));events=[]
 for h in np.arange(.3,t.extents[2],.2):
  prev=s.slice(h-.2);now=s.slice(h);unsupported=now-prev.offset(.21)
  islands=[float(c.area()) for c in now.decompose() if c.area()>.1 and (c^prev.offset(.21)).area()<.01]
  if unsupported.area()>.5 or islands:events.append(dict(h=round(float(h),2),overhang_mm2=unsupported.area(),islands=islands))
 row=dict(part=p['name'],watertight=bool(t.is_watertight),solids=len(t.split()),bed=p['bed'],height=float(t.extents[2]),events=events);rows.append(row)
 print('LAYER SUMMARY',p['name'],'worst',max((e['overhang_mm2'] for e in events),default=0),'islands',[e for e in events if e['islands']],flush=True)
(O/'Current layer audit.json').write_text(json.dumps(rows,indent=2))
print('Current print audit finished.',flush=True)
