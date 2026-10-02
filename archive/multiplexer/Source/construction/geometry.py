from inspect_baseline import *
import manifold3d as m
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
pv=np.array(json.loads((ROOT/'Design parameters.json').read_text())['pivot'])
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(np.asarray(a.vert_properties)[:,:3],np.asarray(a.tri_verts),process=False)
def load(n):return solid(trimesh.load(ROOT/(n+'.stl')))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,a,b,axis=2,center=(0,0,0)):
 s=m.Manifold.cylinder(b-a,r,circular_segments=96)
 if axis==0:s=s.rotate([0,90,0])
 elif axis==1:s=s.rotate([-90,0,0])
 xyz=list(center);xyz[axis]=a;return s.translate(xyz)
def extr(poly,z,h):
 rings=[]
 for g in getattr(poly,'geoms',[poly]):
  if g.is_empty:continue
  rings.append(np.array(g.exterior.coords)[:-1]);rings.extend(np.array(i.coords)[:-1] for i in g.interiors)
 return m.CrossSection(rings,m.FillRule.EvenOdd).extrude(h).translate([0,0,z])
def section(n,z):
 t=trimesh.load(ROOT/(n+'.stl'));s=t.section(plane_origin=[0,0,z],plane_normal=[0,0,1]);p=Polygon()
 for r in s.discrete:p=p.symmetric_difference(Polygon(r[:,:2]))
 return p
