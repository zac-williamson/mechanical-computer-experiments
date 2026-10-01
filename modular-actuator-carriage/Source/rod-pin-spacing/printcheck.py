from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1]/'build'
for name in ['Carriage control rod','Lock control rod']:
 t=trimesh.load_mesh(R/(name+' print.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 oldmesh=trimesh.load_mesh(R/'baseline'/(name+' print.stl'));old=m.Manifold(m.Mesh64(np.ascontiguousarray(oldmesh.vertices),np.ascontiguousarray(oldmesh.faces,dtype=np.uint64)))
 out=[];new_growth=[]
 for h in np.arange(.2,t.extents[2],.2):
  unsupported=s.slice(h)-s.slice(max(.00001,h-.2)).offset(.202,m.JoinType.Round,circular_segments=128)
  prior=old.slice(h)-old.slice(max(.00001,h-.2)).offset(.202,m.JoinType.Round,circular_segments=128)
  added=unsupported-prior.offset(.01)
  if unsupported.area()>prior.area()+.05:new_growth.append([float(h),unsupported.area(),prior.area()])
  if unsupported.area()>.1:out.append([round(float(h),2),round(unsupported.area(),3),unsupported.bounds()])
 print(name,json.dumps(out),flush=True)

 assert not new_growth,new_growth
 assert t.is_watertight and len(t.split())==1
 print('Passed print-oriented layer-area comparison for',name,flush=True)
