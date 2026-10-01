from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'strength-candidate';D=json.loads((O/'Model.json').read_text());report=[]
for p in D['parts']:
 if p['kind']!='printed':continue
 t=trimesh.load(O/(p['name']+'.stl'));ids=np.where((t.area_faces>.15)&(np.max(abs(t.face_normals),axis=1)>.999))[0];points=t.triangles_center[ids];norm=t.face_normals[ids];loc,ray,_=t.ray.intersects_location(points-.001*norm,-norm,multiple_hits=False);dist=np.linalg.norm(loc-points[ray],axis=1);flags=[]
 for j in np.where(dist<1)[0]:
  i=ids[ray[j]];flags.append({'thickness':float(dist[j]),'point':t.triangles_center[i].tolist(),'area':float(t.area_faces[i]),'normal':t.face_normals[i].tolist()})
 flags=sorted(flags,key=lambda a:a['area'],reverse=True);report.append({'part':p['name'],'flags':flags});print(p['name'],json.dumps(flags[:7]),flush=True)
(O/'Thin planar features.json').write_text(json.dumps(report,indent=2))
