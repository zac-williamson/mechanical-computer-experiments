from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
import runpy
clean=runpy.run_path("latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source/clean_print_mesh.py")["clean"]
R=Path(__file__).resolve().parents[1];T=R/'neck-candidate';B=T/'before-guide-strip-removal';B.mkdir(exist_ok=True)
names=['Locking bolt guide.stl','Locking bolt guide print.stl','Model.json','Viewer.html','Print layout.stl','Current geometry hashes.json']
for n in names:
 if (T/n).exists() and not (B/n).exists():shutil.copy2(T/n,B/n)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
old=trimesh.load_mesh(B/'Locking bolt guide.stl');s=solid(old)
cut=box([-20,25.19,46.69],[-12.25,30.11,47.21])+box([2.15,25.19,46.69],[20,30.11,47.21])
out=s-cut;mm=out.to_mesh64();new=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True)
print('mesh',new.is_watertight,[(x.volume,x.is_watertight) for x in new.split(only_watertight=False)],flush=True)
assert new.is_watertight and len(new.split())==1
assert (out-s).volume()<1e-6
removed=(s-out).volume();assert 1<removed<30,removed
new.export(T/'Locking bolt guide.stl',file_type='stl_ascii')
checks=json.loads((T/'Bed orientation checks.json').read_text());p=next(p for p in checks if p['part']=='Locking bolt guide')
printed=new.copy();printed.apply_transform(trimesh.geometry.align_vectors([0,1,0],[0,0,-1]));printed.apply_translation(-printed.bounds[0]);printed.export(T/'Locking bolt guide print.stl',file_type='stl_ascii');placed=printed.copy();placed.apply_translation(p['bounds'][0])
def replace(d,pk,gk,t):
 v=np.frombuffer(gzip.decompress(base64.b64decode(d[gk])),dtype='<f4').reshape(-1,3);arrays=[];offset=0
 for p in d[pk]:
  a=t.triangles.reshape(-1,3).astype('<f4') if p['name']=='Locking bolt guide' else v[p['offset']//3:p['offset']//3+p['vertices']]
  p['offset']=offset;p['vertices']=len(a);offset+=a.size;arrays.append(a)
 d[gk]=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode()
d=json.loads((B/'Model.json').read_text());replace(d,'parts','geometry',new);(T/'Model.json').write_text(json.dumps(d,separators=(',',':')))
shtml=(B/'Viewer.html').read_text();tag='<script type="application/json" id="data">';pre,rest=shtml.split(tag,1);raw,post=rest.split('</script>',1);d=json.loads(raw);replace(d,'parts','geometry',new);replace(d,'print_parts','print_geometry',placed);(T/'Viewer.html').write_text(pre+tag+json.dumps(d,separators=(',',':'))+'</script>'+post)
normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]};parts=[]
for p in checks:
 t=clean(trimesh.load_mesh(T/(p['part']+'.stl')));t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed_face']],[0,0,-1]));t.apply_translation(-t.bounds[0]);t.apply_translation(p['bounds'][0]);parts.append(t)
layout=trimesh.util.concatenate(parts);assert layout.is_watertight;layout.export(T/'Print layout.stl',file_type='stl_ascii')
for n in ['Locking bolt guide.stl','Locking bolt guide print.stl','Print layout.stl']:
 t=trimesh.load_mesh(T/n);assert t.is_watertight,n;print(n,'watertight',flush=True)
report={'removed_mm3':removed,'watertight':bool(new.is_watertight),'solid_count':len(new.split()),'change':'Removed only two 0.5 mm bridges above band openings; guide walls and anchor roots retained. Subtractive change introduces no new interference.'};(T/'Guide strip removal.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
import hashlib
hp=T/'Current geometry hashes.json';h=json.loads(hp.read_text())
for n in names[:-1]:h[n]=hashlib.sha256((T/n).read_bytes()).hexdigest()
hp.write_text(json.dumps(h,indent=2))
