from pathlib import Path
import os,json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'detent-candidate';os.environ['PLANAR_OUTPUT']=str(O)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
# Check both possible bore-vertical print orientations for the extended end.
rows=[]
for name in ['Carriage body','Carriage bearing end']:
 for face,normal in [('left',[-1,0,0]),('right',[1,0,0])]:
  t=trimesh.load(O/(name+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);s=solid(t);islands=[]
  for h in np.arange(.3,t.extents[2],.2):
   sec=s.slice(h);prev=s.slice(h-.2)
   if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):islands.append(round(float(h),2))
  rows.append(dict(part=name,face=face,islands=islands,bed_area=s.slice(.1).area()))
print(rows,flush=True);(O/'Detent print orientation checks.json').write_text(json.dumps(rows,indent=2))
assert not next(r['islands'] for r in rows if r['part']=='Carriage body' and r['face']=='left')
D=json.loads((O/'Model.json').read_text())
for p in D['parts']:
 if p['name']=='Carriage bearing end':
  good=[r for r in rows if r['part']==p['name'] and not r['islands']];assert good,'No island-free orientation';p['bed']=max(good,key=lambda r:r['bed_area'])['face']
(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
runpy.run_path(str(R/'package_preview.py'),run_name='__main__')
s=(R/'export_print_layout.py').read_text().replace("parent/'adapted'","parent/'detent-candidate'");exec(compile(s,'print layout','exec'),{'__name__':'__main__','__file__':str(R/'export_print_layout.py')})
for p in D['parts']:
 if p['name'] in ['Carriage body','Carriage bearing end']:
  t=trimesh.load(O/(p['name']+'.stl'));t.apply_transform(trimesh.geometry.align_vectors([-1,0,0] if p['bed']=='left' else [1,0,0],[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(O/(p['name']+' print.stl'))
runpy.run_path(str(R/'render_review.py'),run_name='__main__')
