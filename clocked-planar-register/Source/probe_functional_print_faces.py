from pathlib import Path
import json,numpy as np,trimesh
O=Path(__file__).resolve().parents[1]/'Functional register';P={p['id']:p for p in json.load(open(O/'parts.json'))};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
for r in json.load(open(O/'Candidate manufacturing checks.json'))['parts']:
 if r['print_geometry_pass']:continue
 p=P[r['part']];v=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(v,np.arange(len(v)).reshape(-1,3),process=True)
 low=t.vertices[:,r['axis']].min() if r['sign']==1 else t.vertices[:,r['axis']].max()
 mask=(t.face_normals[:,r['axis']]*r['sign']<-.7071168)&((t.triangles_center[:,r['axis']]-low)*r['sign']>.001)
 print(r['part'],r['layer_growth_failures'],flush=True)
 for i in np.flatnonzero(mask):print(round(t.area_faces[i],4),np.round(t.triangles[i],4).tolist(),np.round(t.face_normals[i],4).tolist(),flush=True)
