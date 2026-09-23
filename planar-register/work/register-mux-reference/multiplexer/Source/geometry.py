from pathlib import Path
import json,re,gzip,base64,numpy as np,trimesh
import manifold3d as m
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
OUT=Path(__file__).resolve().parents[1]
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(np.asarray(a.vert_properties)[:,:3],np.asarray(a.tri_verts),process=False)
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

def plot(items,path):
 fig,axs=plt.subplots(1,3,figsize=(18,7))
 for ax,(i,j,k,label) in zip(axs,[(0,2,1,'XZ — viewed from negative Y'),(0,1,2,'XY'),(2,1,0,'ZY')]):
  allv=[]
  for p,v in sorted(items,key=lambda a: -a[1][:,k].mean()):
   tri=v.reshape(-1,3,3);order=np.argsort(-tri[:,:,k].mean(1));tri=tri[order]
   co=np.array(p['color'])/255
   ax.add_collection(PolyCollection(tri[:,:,[i,j]],facecolors=[co],edgecolors='none',alpha=1));allv.append(v)
  vv=np.concatenate(allv);ax.set_xlim(vv[:,i].min()-5,vv[:,i].max()+5);ax.set_ylim(vv[:,j].min()-5,vv[:,j].max()+5);ax.set_aspect('equal');ax.set_title(label);ax.set_xlabel('XYZ'[i]+' mm');ax.set_ylabel('XYZ'[j]+' mm');ax.grid(alpha=.2)
  if j==1:ax.invert_yaxis()
 fig.tight_layout();fig.savefig(path,dpi=130);plt.close(fig)
