from pathlib import Path
import trimesh,numpy as np,manifold3d as m
R=Path(__file__).resolve().parent
for n in ['Carriage body','Carriage bearing end']:
 t=trimesh.load(R/'adapted'/(n+'.stl'));e=t.edges_sorted;u,counts=np.unique(e,axis=0,return_counts=True);bad=u[counts!=2];pts=t.vertices[bad];print(n,'bad counts',np.unique(counts[counts!=2],return_counts=True),'bounds',pts.reshape(-1,3).min(0),pts.reshape(-1,3).max(0),flush=True)
 print(np.round(pts[:25],4).tolist(),flush=True)
 s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 print('status',s.status(),'volume',s.volume(),flush=True)
 q=s.simplify(.001);a=q.to_mesh64();r=trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=True);print('simplified',r.volume,r.is_watertight,flush=True)
