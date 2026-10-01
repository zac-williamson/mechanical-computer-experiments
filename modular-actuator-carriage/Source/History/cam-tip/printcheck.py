from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1]/'cam-tip-candidate'
for name in ['Lock control rod']:
 t=trimesh.load_mesh(R/(name+' print.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 oldmesh=trimesh.load_mesh(R/'baseline'/(name+' print.stl'));old=m.Manifold(m.Mesh64(np.ascontiguousarray(oldmesh.vertices),np.ascontiguousarray(oldmesh.faces,dtype=np.uint64)))
 out=[];new_growth=[]
 for h in np.arange(.2,t.extents[2],.2):
  unsupported=s.slice(h)-s.slice(h-.2).offset(.202,m.JoinType.Round,circular_segments=128)
  prior=old.slice(h)-old.slice(h-.2).offset(.202,m.JoinType.Round,circular_segments=128)
  added=unsupported-prior.offset(.01)
  if added.area()>.1:new_growth.append([float(h),added.area()])
  if unsupported.area()>.1:out.append([round(float(h),2),round(unsupported.area(),3),unsupported.bounds()])
 print(name,json.dumps(out),flush=True)

assert not new_growth,new_growth
(R/'Rounded cam print checks.json').write_text(json.dumps(dict(existing_overhang_flags=out,new_overhang_flags=new_growth,layer_mm=.2,limit_degrees_from_vertical=45,scope='No new steep overhangs from rounding. Existing end pin-hole roofs remain flagged; not a whole-part support-free certification.'),indent=2))
import runpy
runpy.run_path(str(Path(__file__).resolve().parent/'render.py'),run_name='__main__')
