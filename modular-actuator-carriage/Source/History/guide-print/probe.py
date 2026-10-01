from pathlib import Path
import trimesh,numpy as np,json
R=Path(__file__).resolve().parents[1]/'neck-candidate'
t=trimesh.load_mesh(R/'Locking bolt guide.stl')
print('bounds',t.bounds)
for ids in t.facets:
 n=t.face_normals[ids[0]];area=t.area_faces[ids].sum()
 if n[2]<-.01 and area>.05:
  print('FACE',np.round(n,3).tolist(),'area',round(area,3),'bounds',np.round(np.array([t.triangles[ids].min((0,1)),t.triangles[ids].max((0,1))]),3).tolist())
