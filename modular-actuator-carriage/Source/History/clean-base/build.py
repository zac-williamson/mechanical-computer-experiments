from pathlib import Path
import shutil,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'clean-base-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for f in A.iterdir():
 if f.is_file() and f.suffix in ['.stl','.json','.html','.md']:
  if not (B/f.name).exists():shutil.copy2(f,B/f.name)
  shutil.copy2(B/f.name,O/f.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,x,z):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([-90,0,0]).translate([x,a,z])
def mesh(s):
 q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True);assert t.is_watertight and len(t.split())==1;return t
# Entirely new frame. No solids imported from the old frame.
s=box([-31.5,44.4,-8],[40.5,48.4,56])
wallpins=[(-24.4,10),(-24.4,38),(24.4,0),(24.4,50)]
for x,z in wallpins:s+=box([x-4,32,z-4.4],[x+4,48.4,z+4.4])
# Independent full-depth connector sockets, open toward the assembly side.
for x in [-26,35]:
 for z in [0,48]:s+=box([x-4.5,40.4,z-4.5],[x+4.5,48.4,z+4.5])
# Guide mounts: same positions and bearing faces as the existing guide.
for x in [-14,14]:s+=box([x-5.25,38.4,39.8],[x+5.25,48.4,50.6])
# Actuator mounting face. Lower bore has an open mouth in front of the backing
# plate: omit the sub-millimetre lip above the cheek's joining-axle clearance.
s+=box([29.5,40.4,29],[40,48.4,43.9])
for x,z in wallpins:s-=cy(2.5,31.9,40.1,x,z)+cy(3.25,31.9,32.4,x,z)
for x in [-26,35]:
 for z in [0,48]:s-=cy(2.5,40.3,48.5,x,z)+cy(3.25,40.3,40.9,x,z)+cy(2.7,48.1,48.5,x,z)
for x in [-14,14]:s-=cy(2.5,38.3,46.4,x,45.05)+cy(3.25,38.3,38.6,x,45.05)
for z in [31.2,39.2]:s-=cy(2.5,40.3,48.5,34.75,z)
t=mesh(s);t.export(O/'Module base.stl',file_type='stl_ascii')
# Prove each wall pin has a socket surrounding its full insertion length.
engagement=[]
for x,z in wallpins:
 sleeve=cy(3.5,32.45,39.95,x,z)-cy(2.55,32.44,39.96,x,z)
 missing=(sleeve-s).volume();assert missing<1e-6,(x,z,missing)
 engagement.append(dict(X=x,Z=z,pin_in_frame_Y=[32,40],engagement_mm=8,verified_surrounding_wall_mm=1,missing_mm3=missing))
(O/'Wall pin engagement checks.json').write_text(json.dumps(engagement,indent=2))
D=json.loads((B/'Model.json').read_text());tag='<script type="application/json" id="data">';h=(B/'Viewer.html').read_text();pre,rr=h.split(tag,1);raw,post=rr.split('</script>',1);V=json.loads(raw)
def unpack(d,k='geometry'):return np.frombuffer(gzip.decompress(base64.b64decode(d[k])),dtype='<f4').reshape(-1,3)
def pack(a):return base64.b64encode(gzip.compress(np.concatenate(a).astype('<f4').tobytes())).decode()
v=unpack(D);arr=[];off=0
for p in D['parts']:
 a=t.triangles.reshape(-1,3) if p['name']=='Module base' else v[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=pack(arr);(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
sv=unpack(V)
for f in V['bands']:
 for p in f.values():
  a=sv[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;off+=a.size;arr.append(a)
V.update(D);V['geometry']=pack(arr)
checks=json.loads((B/'Bed orientation checks.json').read_text());placed={}
for p in checks:
 q=trimesh.load_mesh(B/(p['part']+' print.stl')) if (B/(p['part']+' print.stl')).exists() else None
 if p['part']=='Module base':
  q=t.copy();q.apply_transform(trimesh.geometry.align_vectors([0,1,0],[0,0,-1]));q.apply_translation(-q.bounds[0]);q.export(O/'Module base print.stl',file_type='stl_ascii')
 if q is None:raise RuntimeError(p['part'])
 q.apply_translation(-q.bounds[0]);q.apply_translation(p['bounds'][0]);p['bounds']=q.bounds.tolist();placed[p['part']]=q
pv=unpack(V,'print_geometry');aa=[];off=0
for p in V['print_parts']:
 a=placed[p['name']].triangles.reshape(-1,3);p['offset']=off;p['vertices']=len(a);off+=a.size;aa.append(a)
V['print_geometry']=pack(aa);(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2));trimesh.util.concatenate(list(placed.values())).export(O/'Print layout.stl',file_type='stl_ascii')
pre=pre.replace('72 mm frame candidate','72 mm frame — rebuilt mounting sockets');(O/'Viewer.html').write_text(pre+tag+json.dumps(V,separators=(',',':'))+'</script>'+post)
print('Built new base; all four wall pins have 8 mm sockets.',flush=True)
