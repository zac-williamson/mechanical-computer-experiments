from pathlib import Path
import os,json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'simplified-carriage-r1';A=R/'adapted';os.environ['PLANAR_OUTPUT']=str(O)
assembly=(R/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'simplified-carriage-r1'")
exec(compile(assembly,'assembly checks','exec'),{'__name__':'__main__','__file__':str(R/'carriage-print-split/check_assembly.py')})
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
t=trimesh.load(O/'Carriage bearing end.stl');solid=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
checks=[]
for label,z0,z1 in [('lower gap',11.5,11.6),('upper gap',20.0,20.4)]:
 probe=m.Manifold.cube([8.13,2.28,z1-z0]).translate([-16.24,35.21,z0]);missing=(probe-solid).volume();checks.append({'region':label,'missing_mm3':missing});assert missing<.001
(O/'Recess gap checks.json').write_text(json.dumps(checks,indent=2))
rows=[]
for name,normal in [('Carriage body',[-1,0,0]),('Carriage bearing end',[1,0,0])]:
 t=trimesh.load(O/(name+'.stl'));old=trimesh.load(A/(name+'.stl'));row={'part':name,'old_volume_mm3':float(old.volume),'new_volume_mm3':float(t.volume),'old_planar_patches':len(old.facets),'new_planar_patches':len(t.facets)}
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)));bad=[]
 for h in np.arange(.3,t.extents[2],.2):
  prev=s.slice(h-.2);now=s.slice(h);islands=[c.area() for c in now.decompose() if c.area()>.1 and (c^prev.offset(.21)).area()<.01]
  if islands:bad.append({'height':float(h),'areas':islands})
 row['detached_layer_islands']=bad;rows.append(row);assert not bad,(name,bad)
(O/'Revision print checks.json').write_text(json.dumps(rows,indent=2))
# Generate the current full layout using the same checked bed orientations.
s=(R/'export_print_layout.py').read_text().replace("parent/'adapted'","parent/'simplified-carriage-r1'")
exec(compile(s,'revision print layout','exec'),{'__name__':'__main__','__file__':str(R/'export_print_layout.py')})
runpy.run_path(str(R/'render_review.py'),run_name='__main__')
print('FINAL CHECKS COMPLETE',flush=True)
