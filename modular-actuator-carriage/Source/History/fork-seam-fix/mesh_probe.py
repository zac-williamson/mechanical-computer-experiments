from pathlib import Path
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1]
for n in ['Carriage body','Carriage bearing end']:
 t=trimesh.load_mesh(R/'neck-candidate'/(n+'.stl'));v=t.vertices;v=v[(v[:,2]>4)&(v[:,2]<13.2)&(v[:,1]<13.3)]
 print(n,flush=True)
 for k in range(3):
  u=np.unique(v[:,k]);print('XYZ'[k],u.tolist(),flush=True)
