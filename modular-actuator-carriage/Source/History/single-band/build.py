from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'single-band-candidate';B=O/'baseline';O.mkdir(exist_ok=True);B.mkdir(exist_ok=True)
for p in (R/'neck-candidate').iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json','.md'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def load(folder,name):
 t=trimesh.load_mesh(folder/(name+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,c[0],c[1]])
changes={};report=[]
# Narrow the side portions of the bolt by 1 mm per side. Preserve its
# central bore, follower axle, tip and nose coordinates exactly.
t=trimesh.load_mesh(B/'Locking bolt.stl');x=t.vertices[:,0]+5.05
shift=np.clip((np.abs(x)-3.5)/2.5,0,1)*np.sign(x)
t.vertices[:,0]-=shift
bolt=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
assert bolt.status()==m.Error.NoError
# Remove redundant two-band side hooks and round the two top XZ corners.
bolt=bolt^box([-10.05,23.9,36],[-.05,31.3,52.3])
for x,xa,xb in [(-9.05,-10.06,-9.05),(-1.05,-1.05,-.04)]:
 corner=box([xa,23.9,51.2],[xb,31.3,52.3])
 c=m.Manifold.cylinder(7.4,1,circular_segments=48).rotate([90,0,0]).translate([x,31.3,51.2])
 bolt-=corner-c
changes['Locking bolt']=bolt
guide=load(B,'Locking bolt guide');oldguide=guide
for xa,xb in [(-11.3,-10.3),(.2,1.2)]:
 for ya,yb in [(23.65,25),(30.2,31.55)]:
  guide+=box([xa,ya,39.8],[xb,yb,55.8])
 guide+=box([xa,25,39.8],[xb,30.2,47.2])
# Lift the two existing open anchor pegs a little; retain their stout roots.
from band_geometry import ANCHOR_RISE,ANCHOR_Z,length,mesh
for xa,xb in [(-15,-12.26),(2.16,5)]:
 guide-=box([xa,25.19,40.69],[xb,30.11,45.01])
for a,b,ca,cb in [(-13.8,-12.15,-14.6,-13.8),(2.05,3.7,3.7,4.5)]:
 guide+=cy(1.4,a,b,[27.6,ANCHOR_Z])
 cap=cy(2,ca,cb,[27.6,ANCHOR_Z]).translate([0,-27.6,-ANCHOR_Z]).scale([1,.7,1]).translate([0,27.6,ANCHOR_Z])
 cap=cap^box([-20,20,ANCHOR_Z-1.4],[10,35,60]);guide+=cap
changes['Locking bolt guide']=guide
report.append(dict(part='Locking bolt guide',minimum_side_wall_X_before_mm=.95,minimum_side_wall_X_after_mm=1.95,added_mm3=(guide-oldguide).volume(),anchor_rise_Z_mm=ANCHOR_RISE,anchor_Z_mm=ANCHOR_Z,old_single_loop_length_mm=length(6),new_single_loop_length_mm=length(5,0,ANCHOR_Z),loop_reduction_mm=length(6)-length(5,0,ANCHOR_Z)))
meshes={}
for n,s in changes.items():
 mm=s.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-8]);print('MESH',n,s.status(),[(c.volume,c.is_watertight,c.bounds.tolist()) for c in t.split(only_watertight=False)],flush=True);assert t.is_watertight and len(t.split())==1,n;t.export(O/(n+'.stl'),file_type='stl_ascii');meshes[n]=t
 assert abs(t.volume-s.volume())<.001
 print(n,t.bounds.tolist(),flush=True)
d=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3);d['parts']=[p for p in d['parts'] if not p['name'].startswith('Carriage joining pin ') and p['name']!='Right lock return band']
meshes['Single lock return band']=mesh()
for p in d['parts']:
 if p['name']=='Left lock return band':p['name']='Single lock return band';p['motion']='single-lock-band'
arr=[];offset=0
for p in d['parts']:
 a=meshes[p['name']].triangles.reshape(-1,3).astype('<f4') if p['name'] in meshes else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
d['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(d,separators=(',',':')));(O/'Guide structural changes.json').write_text(json.dumps(report,indent=2));print('BUILD COMPLETE',flush=True)
