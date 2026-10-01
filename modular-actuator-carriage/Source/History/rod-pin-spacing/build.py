from pathlib import Path
import shutil,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'rod-pin-spacing-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for f in A.iterdir():
 if f.is_file() and f.suffix in ['.stl','.json','.html','.md']:
  if not (B/f.name).exists():shutil.copy2(f,B/f.name)
  shutil.copy2(B/f.name,O/f.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,x,z):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([-90,0,0]).translate([x,a,z])
def mesh(s):
 q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True);assert t.is_watertight and len(t.split())==1;return t
def solid(q):return m.Manifold(m.Mesh64(np.ascontiguousarray(q.vertices),np.ascontiguousarray(q.faces,dtype=np.uint64)))
changed={};reports=[]
for name in ['Carriage control rod','Lock control rod']:
 original=trimesh.load_mesh(B/(name+'.stl'));old=solid(original);new=old
 zlo,zhi=(28.2,35.8) if name.startswith('Carriage') else (40,54)
 for x in [-40,-32,32,40]:
  ya,yb=(10.6,14.2) if x<0 else (6.2,9.8)
  region=box([x-3.2,ya,zlo],[x+3.2,yb,zhi])
  void=region-old
  dx=(.5 if abs(x)==32 else -.5)*np.sign(x)
  new=(new+region)-void.translate([dx,0,0])
 q=new.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
 if not t.is_watertight:
  import runpy
  t=runpy.run_path(str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source/clean_print_mesh.py'))['clean'](t)
 assert t.is_watertight and len(t.split())==1,(name,t.is_watertight,len(t.split()))
 final=solid(t);assert final.status()==m.Error.NoError
 assert ((final-new)+(new-final)).volume()<.001
 # Working bearing faces start at X=+/-9.1: cuts stop short of them.
 t.export(O/(name+'.stl'),file_type='stl_ascii');changed[name]=t
 reports.append(dict(part=name,old_faces=len(original.faces),new_faces=len(t.faces),removed_mm3=(old-final).volume(),added_mm3=(final-old).volume(),watertight=True,solids=1))
(O/'Carriage precision checks.json').write_text(json.dumps(reports,indent=2));print(reports,flush=True)
D=json.loads((B/'Model.json').read_text());tag='<script type="application/json" id="data">';h=(B/'Viewer.html').read_text();pre,rr=h.split(tag,1);raw,post=rr.split('</script>',1);V=json.loads(raw)
def unpack(d,k='geometry'):return np.frombuffer(gzip.decompress(base64.b64decode(d[k])),dtype='<f4').reshape(-1,3)
def pack(a):return base64.b64encode(gzip.compress(np.concatenate(a).astype('<f4').tobytes())).decode()
v=unpack(D);arr=[];off=0
for p in D['parts']:
 a=changed[p['name']].triangles.reshape(-1,3) if p['name'] in changed else v[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=pack(arr);(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
sv=unpack(V)
for f in V['bands']:
 for p in f.values():
  a=sv[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;off+=a.size;arr.append(a)
V.update(D);V['geometry']=pack(arr)
T=V['tiling'];tv=unpack(T).copy()
for p in T['parts']:
 if p['name'].startswith('Rod coupling pin'):
  aa=tv[p['offset']//3:p['offset']//3+p['vertices']]
  centre=(aa[:,0].min()+aa[:,0].max())/2
  aa[:,0]+=.5 if centre<36 else -.5
T['geometry']=pack([tv]);(O/'Tiling geometry.json').write_text(json.dumps(T,separators=(',',':')))

checks=json.loads((B/'Bed orientation checks.json').read_text());placed={}
for p in checks:
 q=trimesh.load_mesh(B/(p['part']+' print.stl')) if (B/(p['part']+' print.stl')).exists() else None
 if p['part'] in changed:
  q=changed[p['part']].copy();normal={'right':[1,0,0],'left':[-1,0,0],'bottom':[0,0,-1]}[p['bed_face']];q.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));q.apply_translation(-q.bounds[0]);q.export(O/(p['part']+' print.stl'),file_type='stl_ascii')
 if q is None:raise RuntimeError(p['part'])
 q.apply_translation(-q.bounds[0]);q.apply_translation(p['bounds'][0]);p['bounds']=q.bounds.tolist();placed[p['part']]=q
pv=unpack(V,'print_geometry');aa=[];off=0
for p in V['print_parts']:
 a=placed[p['name']].triangles.reshape(-1,3);p['offset']=off;p['vertices']=len(a);off+=a.size;aa.append(a)
V['print_geometry']=pack(aa);(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2));trimesh.util.concatenate(list(placed.values())).export(O/'Print layout.stl',file_type='stl_ascii')
pre=pre.replace('72 mm module — repaired fork root','72 mm module — fork shoulder removed');(O/'Viewer.html').write_text(pre+tag+json.dumps(V,separators=(',',':'))+'</script>'+post)
print('Updated both carriage meshes.',flush=True)
