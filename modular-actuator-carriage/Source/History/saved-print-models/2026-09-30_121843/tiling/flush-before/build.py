"""Add front-mounted frame bridges; preserve the checked mechanism and rods."""
from pathlib import Path
import json,gzip,base64,shutil,os,runpy,hashlib,sys
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'tile-candidate';O.mkdir(exist_ok=True)
A=R/'adapted';B=R/'tiling'/'before';B.mkdir(exist_ok=True)
for n in ['Model.json','Module base.stl','Viewer.html','README.md']:
 if not (B/n).exists():shutil.copy2(A/n,B/n)
for p in A.iterdir():
 if p.is_file():shutil.copy2(p,O/p.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,x,z):return m.Manifold.cylinder(b-a,r,circular_segments=48).rotate([-90,0,0]).translate([x,a,z])
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 a=s.to_mesh64();t=trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=True);t.vertices=t.vertices.astype('<f4');t=trimesh.Trimesh(t.triangles.reshape(-1,3),np.arange(len(t.faces)*3).reshape(-1,3),process=True)
 assert t.is_watertight and len(t.split())==1
 return t
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
base=solid(trimesh.load(A/'Module base.stl'))
for x in [-32,32]:
 for z in [0,48]:base+=cy(2.51,32,40.2,x,z)+cy(3.26,32,32.41,x,z)
for x in [-34,34]:
 for z in [0,48]:
  base+=box([x-4.5,32,44 if z==48 else z-4.5],[x+4.5,48.4,z+4.5])
  base-=cy(2.5,31.9,40.1,x,z)+cy(3.25,31.9,32.4,x,z)
base=base.simplify(.001);bt=mesh(base);bt.export(O/'Module base.stl')
rodmeshes={}
for name in ['Carriage control rod','Lock control rod']:
 z=32 if name.startswith('Carriage') else 48
 rs=solid(trimesh.load(A/(name+'.stl')))
 for x in [-44,-36]:rs-=cy(2.75,12.6,14.3,x,z)
 rodmeshes[name]=mesh(rs);rodmeshes[name].export(O/(name+'.stl'))
arrays=[];off=0
for p in D['parts']:
 a=bt.triangles.reshape(-1,3) if p['name']=='Module base' else rodmeshes[p['name']].triangles.reshape(-1,3) if p['name'] in rodmeshes else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=off;p['vertices']=len(a);off+=a.size;arrays.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
# Each bridge prints on its broad Y face, with every pin bore vertical.
def bridge(holes):
 s=sum((cy(4.5,24,31.8,x,z) for x,z in holes),m.Manifold()).hull()
 for x,z in holes:s-=cy(2.6,23.9,31.9,x,z)+cy(3.3,31.4,31.9,x,z)
 return s
shapes={'Horizontal frame bridge':bridge([(0,0),(12,0)]),'Vertical frame bridge':bridge([(0,0),(0,16)]),'Centre frame bridge':bridge([(0,0),(12,0),(0,16),(12,16)])}
for name,s in shapes.items():mesh(s).export(O/(name+'.stl'))
T=dict(parts=[],geometry='',pitch=[80,0,64],coupled_rods=True);ta=[];off=0
pinp=next(p for p in D['parts'] if p['name']=='Bearing wall pin -24.4 / 0');aa=np.concatenate(arrays)[pinp['offset']//3:pinp['offset']//3+pinp['vertices']];pin=trimesh.Trimesh(aa,np.arange(len(aa)).reshape(-1,3),process=True);pin.apply_translation([24.4,-32,0])
def add(name,t,kind='printed',motion='fixed',row=0):
 global off
 a=t.triangles.reshape(-1,3);T['parts'].append(dict(name=name,offset=off,vertices=len(a),kind=kind,motion=motion,row=row,color=[.85,.53,.18] if kind=='printed' else [.22,.25,.28]));off+=a.size;ta.append(a)
mounts=[('Horizontal frame bridge',[34,0,0],[(34,0),(46,0)]),('Horizontal frame bridge',[34,0,112],[(34,112),(46,112)]),('Vertical frame bridge',[-34,0,48],[(-34,48),(-34,64)]),('Vertical frame bridge',[114,0,48],[(114,48),(114,64)]),('Centre frame bridge',[34,0,48],[(34,48),(46,48),(34,64),(46,64)])]
for i,(name,pos,holes) in enumerate(mounts):
 add(name+' '+str(i+1),mesh(shapes[name].translate(pos)))
 for x,z in holes:
  t=pin.copy();t.apply_translation([x,31.9,z]);add(f'Frame bridge pin {x}, {z}',t,'native')
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/register-from-multiplexer/Source'))
from ldraw_mesh import LDraw
lib=LDraw('/Applications/Studio 2.0/ldraw/parts/4274.dat');a=lib.mesh()[0].reshape(-1,3)*.4;assert not lib.missing
rod_pin=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);rod_pin.apply_transform(trimesh.geometry.align_vectors([-1,0,0],[0,1,0]))
for row in [0,1]:
 for z,mo in [(32,'carriage'),(48,'lock')]:
  for x in [36,44]:
   t=rod_pin.copy();t.apply_translation([x,5.4,z+64*row]);add(f'Rod coupling pin {row}, {z}, {x}',t,'native',mo,row)
T['geometry']=base64.b64encode(gzip.compress(np.concatenate(ta).astype('<f4').tobytes())).decode();(O/'Tiling geometry.json').write_text(json.dumps(T,separators=(',',':')))
os.environ['PLANAR_OUTPUT']=str(O)
runpy.run_path(str(R/'package_preview.py'),run_name='__main__')
# The tiling meshes have separate buffers; no duplication of the four mechanisms.
h=(O/'Viewer.html').read_text();data=json.loads(h.split('<script type="application/json" id="data">')[1].split('</script>')[0]);data['tiling']=T
start=h.index('<script type="application/json" id="data">')+len('<script type="application/json" id="data">');end=h.index('</script>',start);h=h[:start]+json.dumps(data,separators=(',',':'))+h[end:];(O/'Viewer.html').write_text(h)
print('Built candidate base, three bridge types and 2x2 hardware.',flush=True)
