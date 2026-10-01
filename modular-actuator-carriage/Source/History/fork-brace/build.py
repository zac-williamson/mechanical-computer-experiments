from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'fork-brace-candidate';B=O/'baseline';O.mkdir(exist_ok=True);B.mkdir(exist_ok=True)
for p in (R/'neck-candidate').iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json','.md'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def load(folder,name):
 t=trimesh.load_mesh(folder/(name+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,c[0],c[1]])
changes={};report=[]
# Two narrow ribs brace the fork back to the bearing post. The lower
# ramp rises one millimetre in Z per millimetre of X (45 degrees on bed).
profile=m.CrossSection([[[ -1.6,7.55],[6.95,7.55],[10.7,11.3],[12,11.3],[12,13.05],[10.7,13.05],[6.95,9.3],[-1.6,9.3]]])
braces=m.Manifold()
for ya,yb in [(1.55,3.75),(10.5,12.7)]:
 braces+=profile.extrude(yb-ya).rotate([90,0,0]).translate([0,yb,0])
n='Carriage body';old=load(B,n);new=old+braces;changes[n]=new
report.append(dict(part=n,added_mm3=(new-old).volume(),old_volume_mm3=old.volume(),new_volume_mm3=new.volume(),rib_width_mm=2.2,rib_height_mm=1.75,ramp_degrees=45))
meshes={}
for n,s in changes.items():
 mm=s.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-8]);print('MESH',n,s.status(),[(c.volume,c.is_watertight,c.bounds.tolist()) for c in t.split(only_watertight=False)],flush=True);assert t.is_watertight and len(t.split())==1,n;t.export(O/(n+'.stl'),file_type='stl_ascii');meshes[n]=t
 assert abs(t.volume-s.volume())<.001
 print(n,t.bounds.tolist(),flush=True)
d=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3);d['parts']=[p for p in d['parts'] if not p['name'].startswith('Carriage joining pin ')];arr=[];offset=0
for p in d['parts']:
 a=meshes[p['name']].triangles.reshape(-1,3).astype('<f4') if p['name'] in meshes else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
d['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(d,separators=(',',':')));(O/'Fork brace changes.json').write_text(json.dumps(report,indent=2));print('BUILD COMPLETE',flush=True)
