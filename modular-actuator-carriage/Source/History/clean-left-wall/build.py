from pathlib import Path
import shutil,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'clean-left-wall-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for f in A.iterdir():
 if f.is_file() and f.suffix in ['.stl','.json','.html','.md']:
  if not (B/f.name).exists():shutil.copy2(f,B/f.name)
  shutil.copy2(B/f.name,O/f.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,x,z):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([-90,0,0]).translate([x,a,z])
def mesh(s):
 s=s.simplify(.00001);q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True);assert t.is_watertight and len(t.split())==1,(t.is_watertight,len(t.split()),[(c.volume(),c.bounding_box()) for c in s.decompose()]);return t
# New YZ profile, extruded along X. Lower axle is a round boss tied into
# a diagonal shoulder; a continuous web joins both mounting feet.
profile=[[2.2,7],[3.2,5],[3.2,0],[17.2,0],[23.9,7],[23.9,56],[2.2,56]]
s=m.CrossSection([profile]).extrude(4).rotate([90,0,90]).translate([-24.4,0,0])
s+=m.Manifold.cylinder(4,7,circular_segments=96).rotate([0,90,0]).translate([-24.4,10.2,0])
s+=box([-24.4,23.8,5.6],[-20.4,32,42.4])
for z in [10,38]:s+=box([-28.4,24,z-4.4],[-20.4,32,z+4.4])
# Original four-pad guide clearances, generated parametrically.
for z,lo,hi in [(32,4.45,4.45),(48,8.65,6.65)]:
 u=4.65;w=1.2;d=.3
 poly=[[-u,-lo],[-w,-lo],[-w,-lo+d],[w,-lo+d],[w,-lo],[u,-lo],[u,-w],[u-d,-w],[u-d,w],[u,w],[u,hi],[w,hi],[w,hi-d],[-w,hi-d],[-w,hi],[-u,hi],[-u,w],[-u+d,w],[-u+d,-w],[-u,-w]]
 s-=m.CrossSection([poly]).extrude(4.2).rotate([90,0,90]).translate([-24.5,10.2,z])
for z in [0,16]:
 s-=m.Manifold.cylinder(4.2,2.65,circular_segments=64).rotate([0,90,0]).translate([-24.5,10.2,z])
 s-=m.Manifold.cylinder(.9,3.85,circular_segments=64).rotate([0,90,0]).translate([-24.5,10.2,z])
for z in [10,38]:s-=cy(2.5,23.9,32.1,-24.4,z)+cy(3.25,31.6,32.1,-24.4,z)
t=mesh(s);t.export(O/'Left bearing wall.stl',file_type='stl_ascii')
D=json.loads((B/'Model.json').read_text());tag='<script type="application/json" id="data">';h=(B/'Viewer.html').read_text();pre,rr=h.split(tag,1);raw,post=rr.split('</script>',1);V=json.loads(raw)
def unpack(d,k='geometry'):return np.frombuffer(gzip.decompress(base64.b64decode(d[k])),dtype='<f4').reshape(-1,3)
def pack(a):return base64.b64encode(gzip.compress(np.concatenate(a).astype('<f4').tobytes())).decode()
v=unpack(D);arr=[];off=0
for p in D['parts']:
 a=t.triangles.reshape(-1,3) if p['name']=='Left bearing wall' else v[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=pack(arr);(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
sv=unpack(V)
for f in V['bands']:
 for p in f.values():
  a=sv[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;off+=a.size;arr.append(a)
V.update(D);V['geometry']=pack(arr)
checks=json.loads((B/'Bed orientation checks.json').read_text());placed={}
for p in checks:
 q=trimesh.load_mesh(B/(p['part']+' print.stl')) if (B/(p['part']+' print.stl')).exists() else None
 if p['part']=='Left bearing wall':
  q=t.copy();q.apply_transform(trimesh.geometry.align_vectors([1,0,0],[0,0,-1]));q.apply_translation(-q.bounds[0]);q.export(O/'Left bearing wall print.stl',file_type='stl_ascii')
 if q is None:raise RuntimeError(p['part'])
 q.apply_translation(-q.bounds[0]);q.apply_translation(p['bounds'][0]);p['bounds']=q.bounds.tolist();placed[p['part']]=q
pv=unpack(V,'print_geometry');aa=[];off=0
for p in V['print_parts']:
 a=placed[p['name']].triangles.reshape(-1,3);p['offset']=off;p['vertices']=len(a);off+=a.size;aa.append(a)
V['print_geometry']=pack(aa);(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2));trimesh.util.concatenate(list(placed.values())).export(O/'Print layout.stl',file_type='stl_ascii')
pre=pre.replace('72 mm frame — rebuilt mounting sockets','72 mm module — continuous left bearing wall');(O/'Viewer.html').write_text(pre+tag+json.dumps(V,separators=(',',':'))+'</script>'+post)
print('Built left wall from profile, circular boss and mounting feet.',flush=True)
