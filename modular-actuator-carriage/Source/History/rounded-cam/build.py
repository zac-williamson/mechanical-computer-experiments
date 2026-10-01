from pathlib import Path
import shutil,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'rounded-cam-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
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
for name in ['Lock control rod']:
 original=trimesh.load_mesh(B/(name+'.stl'));old=solid(original)
 # Round the extension's root in plan, then round its outer corners.
 footprint=m.CrossSection.square([40,8]).translate([-20,6.2])+m.CrossSection.square([25,5.5]).translate([-12.5,14.1])
 footprint=footprint.offset(2,m.JoinType.Round,circular_segments=96).offset(-2,m.JoinType.Round,circular_segments=96)
 footprint=footprint.offset(-1.5,m.JoinType.Round,circular_segments=96).offset(1.5,m.JoinType.Round,circular_segments=96)
 profile=old^box([-17,10,39.9],[17,10.1,54.1])
 extension=profile.translate([0,-10,0]).scale([1,55,1]).translate([0,14.1,0])
 extension=extension^footprint.extrude(20).translate([0,0,39])
 new=(old-box([-17,14.200001,39.9],[17,20,54.1]))+extension
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
