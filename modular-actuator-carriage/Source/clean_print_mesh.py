"""Weld STL precision seams within 0.001 mm; remove collapsed triangles."""
import numpy as np
from scipy.spatial import cKDTree
import trimesh

def clean(t):
 t=t.copy()
 # Boolean cuts can leave disconnected zero-thickness surface scraps in STL.
 # Remove only numerically planar, zero-volume debris, never a physical solid.
 pieces=t.split(only_watertight=False)
 if len(pieces)>1:
  keep=[p for p in pieces if not (abs(p.volume)<1e-8 and min(p.extents)<1e-7)]
  if len(keep)<len(pieces):t=trimesh.util.concatenate(keep)
 if t.is_watertight:return t
 pairs=cKDTree(t.vertices).query_pairs(.001);parent=np.arange(len(t.vertices))
 def root(i):
  while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
  return i
 for a,b in pairs:parent[root(b)]=root(a)
 indices=np.array([root(i) for i in range(len(parent))]);t.faces=indices[t.faces]
 t.update_faces(t.unique_faces());t.update_faces(t.nondegenerate_faces(height=1e-8));t.remove_unreferenced_vertices()
 trimesh.repair.fill_holes(t)
 return t
