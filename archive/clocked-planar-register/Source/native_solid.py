"""Repair LDraw T-junction topology without replacing teeth or filling bores.

LDraw subfiles meet along subdivided edges. Split a face at existing vertices
on its edges, retaining the original surface (within stated seam tolerance).
No convex hull, voxelisation, hole fill or implicit collision exemption.
"""
import numpy as np,trimesh
from scipy.spatial import cKDTree

def repair(mesh,tolerance=2e-4):
 t=mesh.copy();t.merge_vertices(digits_vertex=4);t.update_faces(t.unique_faces());t.remove_unreferenced_vertices()
 before=t.vertices.copy();added=0
 for iteration in range(3):
  vv=t.vertices.copy();tree=cKDTree(vv);vertices=vv.tolist();faces=[];split_count=0
  for tri in t.faces:
   boundary=[]
   for ia,ib in zip(tri,np.roll(tri,-1)):
    a,b=vv[ia],vv[ib];d=b-a;length=np.linalg.norm(d)
    boundary.append(int(ia))
    if length<tolerance:continue
    ids=tree.query_ball_point((a+b)/2,length/2+tolerance)
    pts=vv[ids];u=(pts-a)@d/(length*length);distance=np.linalg.norm(pts-a-u[:,None]*d,axis=1)
    valid=(u>tolerance/length)&(u<1-tolerance/length)&(distance<tolerance)
    boundary.extend(int(ids[j]) for j in np.flatnonzero(valid)[np.argsort(u[valid])])
   if len(boundary)==3:faces.append(list(tri));continue
   split_count+=1;centre=len(vertices);vertices.append(vv[tri].mean(axis=0).tolist())
   faces.extend([[centre,a,b] for a,b in zip(boundary,boundary[1:]+boundary[:1])])
  if not split_count:break
  added+=split_count;t=trimesh.Trimesh(np.array(vertices),np.array(faces),process=True)
  t.update_faces(t.unique_faces());t.remove_unreferenced_vertices()
  if t.is_watertight:break
 trimesh.repair.fix_normals(t,multibody=True)
 edges,counts=np.unique(t.edges_sorted,axis=0,return_counts=True)
 return t,dict(seam_tolerance_mm=tolerance,faces_split=added,watertight=bool(t.is_watertight),boundary_edges=int(sum(counts==1)),nonmanifold_edges=int(sum(counts>2)))

if __name__=='__main__':
 import json
 from pathlib import Path
 import manifold3d as m
 R=Path(__file__).resolve().parents[1]/'Compact layout';ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
 for name in ['master_gate L099','master_gate L102','master L097','master U022']:
  p=next(p for p in ps if p['id']==name);a=v[p['offset']//3:p['offset']//3+p['vertices']];t,report=repair(trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True));print(name,report,flush=True)
  if t.is_watertight:print('manifold',m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64))).status(),flush=True)
