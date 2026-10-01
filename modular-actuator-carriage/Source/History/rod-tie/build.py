from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'rod-tie-candidate';B=O/'baseline';O.mkdir(exist_ok=True);B.mkdir(exist_ok=True)
for p in (R/'neck-candidate').iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json','.md'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def load(folder,name):
 t=trimesh.load_mesh(folder/(name+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,c[0],c[1]])
changes={};report=[]
for n,xa,xb in [('Carriage body',-1.6,16.25),('Carriage bearing end',-16.25,-1.8)]:
 old=load(B,n);before=load(R/'neck-candidate/before-open-carriage',n)
 mounts=box([xa-.01,-7,6.7],[xb+.01,4.01,27.8])
 # Restore only the pre-mount surfaces inside the old mount region.
 new=(old-mounts)+(before^mounts)
 # Open locating seats through the inner split face: no thin inner wall.
 sx,ex=(-1.61,6.4) if n=='Carriage body' else (-6.4,-1.79)
 seat=m.CrossSection([[(14.1,29.8),(16.45,32.15),(16.45,35.95),(14.1,35.95)]]).extrude(ex-sx).rotate([90,0,90]).translate([sx,0,0])
 new-=seat;changes[n]=new
 report.append(dict(part=n,old_Y_min=old.bounding_box()[1],new_Y_min=new.bounding_box()[1],removed_mm3=(old-new).volume(),added_mm3=(new-old).volume()))
rod=load(B,'Carriage control rod')
for xa,xb in [(-6.25,-2.75),(2.75,6.25)]:
 ramp=m.CrossSection([[(14.1,30.1),(14.2,30.1),(16.2,32.1),(16.2,35.8),(14.1,35.8)]]).extrude(xb-xa).rotate([90,0,90]).translate([xa,0,0]);rod+=ramp
changes['Carriage control rod']=rod
meshes={}
for n,s in changes.items():
 mm=s.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-8]);print('MESH',n,s.status(),[(c.volume,c.is_watertight,c.bounds.tolist()) for c in t.split(only_watertight=False)],flush=True);assert t.is_watertight and len(t.split())==1,n;t.export(O/(n+'.stl'),file_type='stl_ascii');meshes[n]=t
 assert abs(t.volume-s.volume())<.001
 print(n,t.bounds.tolist(),flush=True)
d=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3);d['parts']=[p for p in d['parts'] if not p['name'].startswith('Carriage joining pin ')];arr=[];offset=0
for p in d['parts']:
 a=meshes[p['name']].triangles.reshape(-1,3).astype('<f4') if p['name'] in meshes else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
d['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(d,separators=(',',':')));(O/'Rod tie changes.json').write_text(json.dumps(report,indent=2));print('BUILD COMPLETE',flush=True)
