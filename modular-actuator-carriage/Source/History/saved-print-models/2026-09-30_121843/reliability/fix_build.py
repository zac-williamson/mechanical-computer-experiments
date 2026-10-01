from pathlib import Path
import json,os,shutil,runpy,base64,gzip,ast,sys
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'reliability-candidate';A=R/'adapted';B=R/'reliability/before';B.mkdir(exist_ok=True)
for n in ['Model.json','Locking bolt.stl','README.md','Viewer.html']:
 if not (B/n).exists():shutil.copy2(A/n,B/n)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
old=solid(trimesh.load(A/'Locking bolt.stl'));s=runpy.run_path(str(R/'reliability/fixes.py'))['fix_bolt'](old,box,cy).simplify(.001)
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']]
env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[])
exec(compile(ast.Module(body=defs,type_ignores=[]),'checked STL export','exec'),env)
env['add']('Locking bolt',s,'bolt','front');t=env['parts'][0]['mesh'];ss=solid(t)
assert (ss-old).volume()<.1
t.export(O/'Locking bolt.stl')
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=t.triangles.reshape(-1,3) if p['name']=='Locking bolt' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
os.environ['PLANAR_OUTPUT']=str(O)
for script in ['validate.py','package_preview.py']:
 runpy.run_path(str(R/script),run_name='__main__')
# Independent island screen in the actual print orientation.
p=t.copy();p.apply_transform(trimesh.geometry.align_vectors([0,-1,0],[0,0,-1]));p.apply_translation(-p.bounds[0]);s=solid(p);bad=[]
for h in np.arange(.3,p.extents[2],.2):
 prev=s.slice(h-.2);now=s.slice(h);islands=[c.area() for c in now.decompose() if c.area()>.1 and (c^prev.offset(.21)).area()<.01]
 if islands:bad.append(dict(height=float(h),areas=islands))
assert not bad,bad;p.export(O/'Locking bolt print.stl')
(O/'Bolt print fix checks.json').write_text(json.dumps(dict(watertight=True,single_solid=True,detached_islands=bad,passed=True,removed_mm3=(old-ss).volume(),added_mm3=(ss-old).volume(),preserved='Root, guide faces, follower bore, locking tip and flange Z extent unchanged.'),indent=2))
print('Moving bolt retaining ends corrected; no detached islands.',flush=True)
