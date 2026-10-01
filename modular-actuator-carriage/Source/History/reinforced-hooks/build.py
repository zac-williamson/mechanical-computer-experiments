from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'reinforced-hooks-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for p in (R/'neck-candidate').iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json','.md'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def hull(points):return m.Manifold.hull_points(points)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def hole(x,r,y0,y1):
 cyl=m.Manifold.cylinder(y1-y0,r,circular_segments=64).rotate([-90,0,0]).translate([x,y0,45.05])
 roof=hull([[x+dx,y,45.05+z] for dx,z in [(-r/2**.5,r/2**.5),(0,r*2**.5),(r/2**.5,r/2**.5)] for y in [y0,y1]])
 return cyl+roof
# Straight walls, a rear mounting plate and two pairs of retaining rails.
g=box([-19.9,32.6,39.8],[19.9,38,55.8])
g+=box([-12.25,20,39.8],[2.15,23.8,55.8])
for xa,xb in [(-12.25,-10.3),(.2,2.15)]:g+=box([xa,23.65,39.8],[xb,32.61,55.8])
g+=box([-12.25,31.4,39.8],[2.15,32.61,55.8])
# Open band entrances above the hook roots, between front and rear guide rails.
g-=box([-12.26,25,47.2],[2.16,30.2,56])
# Open follower-axle and rear-bush path. The nose remains enclosed below it.
g-=box([-9.05,19,43.9],[-1.05,39,56])
for x in [-14,14]:
 g-=hole(x,2.5,32.59,38.1)
 g-=hole(x,3.25,37.8,38.1)
# Cantilever hooks grow at 45 degrees in X. Flange grows at 45 degrees in Y.
from band_geometry import ROOT_Z,mesh
for root,sgn in [(-12.25,-1),(2.15,1)]:
 tip=root+sgn*3.15
 pts=[[x,y,z] for x,z in [(root,ROOT_Z),(tip,ROOT_Z+3.15),(tip,45.0),(root,45.0)] for y in [26.2,29.0]]
 g+=hull(pts)
 cap=hull([[root+sgn*d,y,z] for d in [1.55,3.15] for y,z in [(26.2,ROOT_Z+d),(29,ROOT_Z+d),(29.4,ROOT_Z+d+.4),(29.4,46.5),(25.8,46.5),(25.8,ROOT_Z+d+.4)]])
 g+=cap
mm=g.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);assert t.is_watertight and len(t.split())==1;t.export(O/'Locking bolt guide.stl',file_type='stl_ascii')
D=json.loads((B/'Model.json').read_text());D['lock_band_geometry']='support-free-guide/band_geometry.py';v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];off=0
for p in D['parts']:
 a=t.triangles.reshape(-1,3).astype('<f4') if p['name']=='Locking bolt guide' else mesh().triangles.reshape(-1,3).astype('<f4') if p['name']=='Single lock return band' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
bad=(t.face_normals[:,2]<-.70711)&(t.triangles[:,:,2].max(1)>39.80001)
print('mesh',g.status(),'volume',g.volume(),'steep face area',t.area_faces[bad].sum(),flush=True)
assert t.area_faces[bad].sum()<.001
