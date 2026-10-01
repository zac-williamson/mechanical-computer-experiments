from pathlib import Path
import json,runpy,sys
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'flat-face-candidate';A=R/'neck-candidate'
d=json.loads((O/'Clearance checks.json').read_text());assert not any(d[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'));from clean_print_mesh import clean
rows=[]
for n,normal in [('Carriage body',[-1,0,0]),('Carriage bearing end',[1,0,0])]:
 t=clean(trimesh.load(O/(n+'.stl')));t.export(O/(n+'.stl'));t=trimesh.load(O/(n+'.stl'));assert t.is_watertight and len(t.split())==1
 def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 old=solid(trimesh.load(A/(n+'.stl')));s=solid(t);removed=(old-s).volume();assert removed>=0
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);s=solid(t);bad=[]
 for h in np.arange(.3,t.extents[2],.2):
  prev=s.slice(h-.2);now=s.slice(h)
  if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in now.decompose()):bad.append(float(h))
 assert not bad;rows.append(dict(part=n,watertight=True,solids=1,removed_mm3=removed,detached_layer_islands=bad))
(O/'Flat face verification.json').write_text(json.dumps(rows,indent=2))
runpy.run_path(str(R/'flat-face-study/view.py'),run_name='__main__')
print('Flat face checked and viewer ready',flush=True)
