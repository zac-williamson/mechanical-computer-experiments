from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'visible-carriage-candidate';B=O/'baseline';O.mkdir(exist_ok=True);B.mkdir(exist_ok=True)
for p in (R/'neck-candidate').iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json','.md'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def load(folder,name):
 t=trimesh.load_mesh(folder/(name+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,c[0],c[1]])
changes={};report=[]
# One radiused sight opening; retain the two bearing posts and the fork beam.
radius=2.0
profile=m.CrossSection.square([14,12.7]).translate([-7,13.3]).offset(radius,circular_segments=32)
window=profile.extrude(15).rotate([90,0,0]).translate([0,14,0])
for n in ['Carriage body','Carriage bearing end']:
 old=load(B,n);new=old-window;changes[n]=new
 report.append(dict(part=n,removed_mm3=(old-new).volume(),old_volume_mm3=old.volume(),new_volume_mm3=new.volume(),opening_X=[-9,9],opening_Z=[11.3,28],cut_Y=[-1,14],corner_radius=2))
meshes={}
for n,s in changes.items():
 mm=s.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-8]);print('MESH',n,s.status(),[(c.volume,c.is_watertight,c.bounds.tolist()) for c in t.split(only_watertight=False)],flush=True);assert t.is_watertight and len(t.split())==1,n;t.export(O/(n+'.stl'),file_type='stl_ascii');meshes[n]=t
 assert abs(t.volume-s.volume())<.001
 print(n,t.bounds.tolist(),flush=True)
d=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3);d['parts']=[p for p in d['parts'] if not p['name'].startswith('Carriage joining pin ')];arr=[];offset=0
for p in d['parts']:
 a=meshes[p['name']].triangles.reshape(-1,3).astype('<f4') if p['name'] in meshes else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
d['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(d,separators=(',',':')));(O/'Visibility changes.json').write_text(json.dumps(report,indent=2));print('BUILD COMPLETE',flush=True)
