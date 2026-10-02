import numpy as np,trimesh
from shapely.geometry import MultiPoint
from motion import lift

def lock_band(q,clearance=0):
 pts=[(-17,38),(3,38),(-7,28.5+lift(q))];h=MultiPoint(pts).convex_hull
 ring=h.buffer(3.5+clearance,quad_segs=16).difference(h.buffer(2.9-clearance,quad_segs=16))
 # Extrude triangulated ring using manifold3d, avoiding an external triangulator.
 import manifold3d as m
 loops=[np.asarray(ring.exterior.coords)[:-1]]+[np.asarray(x.coords)[:-1] for x in ring.interiors]
 cs=m.CrossSection(loops,m.FillRule.EvenOdd);a=cs.extrude(1.2+2*clearance).to_mesh64();t=trimesh.Trimesh(np.asarray(a.vert_properties)[:,:3],np.asarray(a.tri_verts),process=True)
 v=t.vertices.copy();t.vertices=np.column_stack([v[:,0],v[:,2]-27.4-clearance,v[:,1]])
 t.invert();return t
