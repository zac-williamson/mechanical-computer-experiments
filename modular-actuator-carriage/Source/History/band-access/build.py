from pathlib import Path
import runpy,os,json,base64,gzip,shutil,sys
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'adapted';O=R/'band-candidate';B=R/'band-access/before';O.mkdir(exist_ok=True);B.mkdir(exist_ok=True)
for p in A.iterdir():
 if p.is_file():shutil.copy2(p,O/p.name)
for name in ['Model.json','Locking bolt guide.stl','Viewer.html','README.md']:
 if not (B/name).exists():shutil.copy2(A/name,B/name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
t=trimesh.load(A/'Locking bolt guide.stl');old=solid(t)
s=runpy.run_path(str(R/'band-access/open_anchors.py'))['open_anchors'](old,box,cy).simplify(.001)
a=s.to_mesh64();t=trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=True);t.vertices=t.vertices.astype('<f4');t=trimesh.Trimesh(t.triangles.reshape(-1,3),np.arange(len(t.faces)*3).reshape(-1,3),process=True)
clean=runpy.run_path(str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source/clean_print_mesh.py'))['clean'];t=clean(t)
assert t.is_watertight and len(t.split())==1
ss=solid(t);assert abs((s-ss).volume())+abs((ss-s).volume())<.1
assert (ss-old).volume()<.1,'Only removing enclosing material and narrowing end flanges'
t.export(O/'Locking bolt guide.stl')
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arrays=[];off=0
for p in D['parts']:
 if p['name']=='Locking bolt guide':p['bed']='rear'
 a=t.triangles.reshape(-1,3) if p['name']=='Locking bolt guide' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=off;p['vertices']=len(a);off+=a.size;arrays.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
os.environ['PLANAR_OUTPUT']=str(O)
for script in ['validate.py','package_preview.py']:
 runpy.run_path(str(R/script),run_name='__main__')
print('Open band anchors built and operating clearances checked.',flush=True)
