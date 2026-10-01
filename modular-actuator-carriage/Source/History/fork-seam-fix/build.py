from pathlib import Path
import shutil,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'fork-seam-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for f in A.iterdir():
 if f.is_file() and f.suffix in ['.stl','.json','.html','.md']:
  if not (B/f.name).exists():shutil.copy2(f,B/f.name)
  shutil.copy2(B/f.name,O/f.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,x,z):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([-90,0,0]).translate([x,a,z])
def mesh(s):
 q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
 if not t.is_watertight:
  import runpy
  t=runpy.run_path(str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source/clean_print_mesh.py'))['clean'](t)
 assert t.is_watertight and len(t.split())==1
 cleaned=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 assert cleaned.status()==m.Error.NoError
 delta=(s-cleaned).volume()+(cleaned-s).volume();assert delta<.001,delta
 return t
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
old=solid(trimesh.load_mesh(B/'Carriage body.stl'))
# Preserve the working fork below the root. Discard the numerical slivers at its edges.
fork=old^box([-1.6,1.54,3.9],[1.6,13.21,7.55])
region=box([-1.61,1.54,3.9],[9.099,13.21,11.3001])
root_profile=[[-1.6,1.55],[1.6,1.55],[1.6,12.7],[-1.6,12.7]]
root=m.CrossSection([root_profile]).extrude(1.95).translate([0,0,7.35])
profile=m.CrossSection([[[-1.6,7.55],[6.95,7.55],[10.7,11.3],[12,11.3],[12,13.05],[10.7,13.05],[6.95,9.3],[-1.6,9.3]]])
ribs=m.Manifold()
for ya,yb in [(1.55,3.75),(10.5,12.7)]:ribs+=profile.extrude(yb-ya).rotate([90,0,0]).translate([0,yb,0])
shoulder=box([-1.6,1.55,9.1],[1.6,4.2,11.3])
s=(old-region)+fork+root+ribs+shoulder
t=mesh(s);t.export(O/'Carriage body.stl',file_type='stl_ascii')
(O/'Fork seam changes.json').write_text(json.dumps(dict(removed_mm3=(old-s).volume(),added_mm3=(s-old).volume(),fork_root_overlap_Z_mm=.2,root_Z=[7.35,9.3],rib_root_intersection_mm3=(root^ribs).volume(),fork_root_intersection_mm3=(fork^root).volume()),indent=2))
D=json.loads((B/'Model.json').read_text());tag='<script type="application/json" id="data">';h=(B/'Viewer.html').read_text();pre,rr=h.split(tag,1);raw,post=rr.split('</script>',1);V=json.loads(raw)
def unpack(d,k='geometry'):return np.frombuffer(gzip.decompress(base64.b64decode(d[k])),dtype='<f4').reshape(-1,3)
def pack(a):return base64.b64encode(gzip.compress(np.concatenate(a).astype('<f4').tobytes())).decode()
v=unpack(D);arr=[];off=0
for p in D['parts']:
 a=t.triangles.reshape(-1,3) if p['name']=='Carriage body' else v[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=pack(arr);(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
sv=unpack(V)
for f in V['bands']:
 for p in f.values():
  a=sv[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;off+=a.size;arr.append(a)
V.update(D);V['geometry']=pack(arr)
checks=json.loads((B/'Bed orientation checks.json').read_text());placed={}
for p in checks:
 q=trimesh.load_mesh(B/(p['part']+' print.stl')) if (B/(p['part']+' print.stl')).exists() else None
 if p['part']=='Carriage body':
  q=t.copy();q.apply_transform(trimesh.geometry.align_vectors([1,0,0],[0,0,-1]));q.apply_translation(-q.bounds[0]);q.export(O/'Carriage body print.stl',file_type='stl_ascii')
 if q is None:raise RuntimeError(p['part'])
 q.apply_translation(-q.bounds[0]);q.apply_translation(p['bounds'][0]);p['bounds']=q.bounds.tolist();placed[p['part']]=q
pv=unpack(V,'print_geometry');aa=[];off=0
for p in V['print_parts']:
 a=placed[p['name']].triangles.reshape(-1,3);p['offset']=off;p['vertices']=len(a);off+=a.size;aa.append(a)
V['print_geometry']=pack(aa);(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2));trimesh.util.concatenate(list(placed.values())).export(O/'Print layout.stl',file_type='stl_ascii')
pre=pre.replace('72 mm module — continuous left bearing wall','72 mm module — repaired fork root');(O/'Viewer.html').write_text(pre+tag+json.dumps(V,separators=(',',':'))+'</script>'+post)
print('Built fork-root repair with preserved working fork geometry.',flush=True)
