from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'compact-frame-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for f in A.iterdir():
 if f.is_file() and f.suffix in ['.stl','.json','.html','.md']:
  if not (B/f.name).exists():shutil.copy2(f,B/f.name)
  shutil.copy2(B/f.name,O/f.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,x,z):return m.Manifold.cylinder(b-a,r,circular_segments=48).rotate([-90,0,0]).translate([x,a,z])
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def get(n):return solid(trimesh.load_mesh(B/(n+'.stl')))
def mesh(s):
 a=s.to_mesh64();t=trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=True);assert s.status()==m.Error.NoError and t.is_watertight and len(t.split())==1, (str(s.status()),t.is_watertight,len(t.split()))
 return t
changed={}
def save(n,s):
 t=mesh(s);t.export(O/(n+'.stl'),file_type='stl_ascii');changed[n]=t
base=get('Module base')
# Restore old joining pockets before relocating them.
for x in [-34,34]:
 for z in [0,48]:
  xa,xb=(-40,-29.3) if x<0 else (29.3,40)
  za,zb=(-8,4.7) if z==0 else (43.8,56)
  base+=box([xa,32,za],[xb,40.41,zb])+cy(2.51,40.3,48.4,x,z)+cy(3.26,40.3,40.9,x,z)
for z in [0,50]:base+=cy(2.6,32,40.2,-24.4,z)+cy(3.3,32,32.5,-24.4,z)
base=base^box([-31.5,31.9,-8],[40,48.5,56])
base+=box([39.9,44.45,-8],[40.5,48.4,56])
for z0,z1 in [(-8,4.8),(44,56)]:base+=box([39.9,32,z0],[40.5,48.4,z1])
for x in [-26,35]:
 for z in [0,48]:
  xa,xb=(-31.6,-21.3) if x<0 else (30.3,40.6)
  za,zb=(-8.1,4.7) if z==0 else (43.8,56.1)
  base-=box([xa,31.9,za],[xb,40.4,zb])
  base-=cy(2.5,40.3,48.5,x,z)+cy(3.25,40.3,40.9,x,z)+cy(2.7,48.1,48.5,x,z)
for z in [10,38]:base-=cy(2.5,31.9,40.1,-24.4,z)+cy(3.25,31.9,32.4,-24.4,z)
base-=get('Upper actuator cheek')+get('Upper actuator cheek').translate([0,.2,0])
print('BASE COMPONENTS',[(c.volume(),c.bounding_box()) for c in base.decompose()],flush=True)
save('Module base',base)
wall=get('Left bearing wall')
for z in [0,50]:wall+=cy(2.6,24,32,-24.4,z)+cy(3.3,31.5,32,-24.4,z)
wall+=box([-28.4,24,5.6],[-20.4,32,14.4])
for z in [10,38]:wall-=cy(2.5,23.9,32.1,-24.4,z)+cy(3.25,31.6,32.1,-24.4,z)
# Open rear access for bridges and their pins; bearing/rod surfaces are at smaller Y.
for za,zb in [(-8,4.7),(43.8,56.1)]:wall-=box([-28.5,23.9,za],[-20.3,32.1,zb])
# Connector ends reach X +/-28; recess the outer bearing mouths by 0.8 mm.
for name,s,lo,hi in [('Left bearing wall',wall,-28.5,-27.6),('Right bearing wall',get('Right bearing wall'),27.6,28.5)]:
 for z in [0,16]:s-=m.Manifold.cylinder(hi-lo,4.05,circular_segments=64).rotate([0,90,0]).translate([lo,10.2,z])
 # Seat the existing 4 mm half-bush 0.8 mm farther inward, leaving 0.2 mm to the connector.
 xa,xb=(-24.5,-23.6) if name.startswith('Left') else (23.6,24.5)
 for z in [0,16]:s-=m.Manifold.cylinder(xb-xa,3.85,circular_segments=64).rotate([0,90,0]).translate([xa,10.2,z])
 save(name,s)
# Shorten each straight section by 4 mm, carrying the existing end geometry inward.
for name in ['Carriage control rod','Lock control rod']:
 s=get(name);mid=s^box([-28.01,-100,-100],[28.01,100,100]);left=(s^box([-100,-100,-100],[-32,100,100])).translate([4,0,0]);right=(s^box([32,-100,-100],[100,100,100])).translate([-4,0,0]);save(name,mid+left+right)
D=json.loads((B/'Model.json').read_text())
def unpack(d):return np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3)
def pack(a):return base64.b64encode(gzip.compress(np.concatenate(a).astype('<f4').tobytes())).decode()
v=unpack(D);arr=[];off=0
for p in D['parts']:
 a=changed[p['name']].triangles.reshape(-1,3) if p['name'] in changed else v[p['offset']//3:p['offset']//3+p['vertices']].copy()
 if p['name'].startswith('Bearing wall pin -24.4'):
  a[:,2]+=10 if p['name'].endswith('/ 0') else -12
  # Retain identifiers for existing viewer and validation hooks.
 if p['name'] in ['Left input axle retainer','L105']:a[:,0]+=.8
 if p['name'] in ['Right input axle retainer','L069']:a[:,0]-=.8
 if p['name']=='Left output axle 4L':a[:,0]+=2
 if p['name']=='Right output axle 4L':a[:,0]-=2
 p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=pack(arr);(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
# Rebuild all connecting bridges and hardware for 72 mm X pitch.
oldT=json.loads((B/'Tiling geometry.json').read_text());tv=unpack(oldT);T=dict(parts=[],geometry='',pitch=[72,0,64],coupled_rods=True);ta=[];toff=0
pin=next(p for p in D['parts'] if p['name']=='Bearing wall pin -24.4 / 0');pin_a=np.concatenate(arr)[pin['offset']//3:pin['offset']//3+pin['vertices']].copy();pin_a-=np.array([-24.4,32,10])
def add(name,a,kind='printed',motion='fixed',row=0):
 global toff
 T['parts'].append(dict(name=name,offset=toff,vertices=len(a),kind=kind,motion=motion,row=row,color=[.85,.53,.18] if kind=='printed' else [.35,.38,.4]));toff+=a.size;ta.append(a)
def bridge(holes):
 s=sum((cy(4.5,32.5,40.3,x,z) for x,z in holes),m.Manifold()).hull()
 # Flat underside preserves 1.2 mm below the 2.6 mm-radius hole.
 s=s^box([-100,32.4,min(z for x,z in holes)-3.8],[160,40.4,max(z for x,z in holes)+4.5])
 for x,z in holes:s-=cy(2.6,32.4,40.4,x,z)+cy(3.3,39.9,40.4,x,z)
 return s
specs=[('Horizontal frame bridge',[(0,0),(11,0)]),('Vertical frame bridge',[(0,0),(0,16)]),('Centre frame bridge',[(0,0),(11,0),(0,16),(11,16)])]
for n,h in specs:mesh(bridge(h)).export(O/(n+'.stl'),file_type='stl_ascii')
mounts=[('Horizontal frame bridge',[(35,0),(46,0)]),('Horizontal frame bridge',[(35,112),(46,112)]),('Vertical frame bridge',[(-26,48),(-26,64)]),('Vertical frame bridge',[(107,48),(107,64)]),('Centre frame bridge',[(35,48),(46,48),(35,64),(46,64)])]
for i,(n,h) in enumerate(mounts):
 add(n+' '+str(i+1),mesh(bridge(h)).triangles.reshape(-1,3))
 for x,z in h:add(f'Frame bridge pin {x}, {z}',pin_a+np.array([x,40.4,z]),'native')
for p in oldT['parts']:
 if not ('Rod coupling pin' in p['name'] or '2L axle connector' in p['name']):continue
 a=tv[p['offset']//3:p['offset']//3+p['vertices']].copy();a[:,0]-=4
 add(p['name'],a,p['kind'],p['motion'],p['row'])
T['geometry']=pack(ta);(O/'Tiling geometry.json').write_text(json.dumps(T,separators=(',',':')))
# Preserve the existing viewer, pose data and print placements.
tag='<script type="application/json" id="data">';h=(B/'before-pin-depth-overlay/Viewer.html').read_text() if (B/'before-pin-depth-overlay/Viewer.html').exists() else (A/'before-pin-depth-overlay/Viewer.html').read_text();pre,rest=h.split(tag,1);raw,post=rest.split('</script>',1);V=json.loads(raw);sv=unpack(V)
for f in V['bands']:
 for p in f.values():
  a=sv[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=off;off+=a.size;arr.append(a)
V.update(D);V['geometry']=pack(arr);V['tiling']=T
normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]};checks=json.loads((B/'Bed orientation checks.json').read_text());placed={}
for p in checks:
 n=p['part'];t=trimesh.load_mesh(O/(n+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed_face']],[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(O/(n+' print.stl'),file_type='stl_ascii');t.apply_translation(p['bounds'][0]);p['bounds']=t.bounds.tolist();placed[n]=t
pa=[];po=0
for p in V['print_parts']:
 a=placed[p['name']].triangles.reshape(-1,3);p['offset']=po;p['vertices']=len(a);po+=a.size;pa.append(a)
V['print_geometry']=pack(pa);trimesh.util.concatenate(list(placed.values())).export(O/'Print layout.stl',file_type='stl_ascii');(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2))
post=post.replace('[80,0,index]','[72,0,index]').replace('[80,64,upperIndex]','[72,64,upperIndex]').replace('[40,22,56]','[36,22,56]').replace('160 mm X × 128 mm Z','144 mm X × 128 mm Z')
pre=pre.replace('Reinforced band hooks','72 mm frame candidate').replace('Thicker retaining lips · taller band stops · 45° print slopes · supports off.','72 × 64 mm footprint · unchanged Y depth · relocated left wall pins · 9L / 4L axles. Candidate under validation.')
(O/'Viewer.html').write_text(pre+tag+json.dumps(V,separators=(',',':'))+'</script>'+post)
print('Built compact candidate',list(changed),flush=True)
