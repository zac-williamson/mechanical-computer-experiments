from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'lock-guidance-candidate';O.mkdir(exist_ok=True);B=O/'baseline';B.mkdir(exist_ok=True)
for p in A.iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def solid(n):
 t=trimesh.load_mesh(B/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def rect(cx,cy,wx,wy,z):return box([cx-wx/2,cy-wy/2,z],[cx+wx/2,cy+wy/2,z+.001])
guide=solid('Locking bolt guide')
for a,b in [(-13.45,-9.05),(-1.05,3.35)]:
 guide+=box([a,20,43.5],[b,23.8,55.8])+box([a,31.4,43.5],[b,34.5,55.8])
for a,b in [(-13.45,-11.3),(1.2,3.35)]:
 for ya,yb in [(23.65,25),(30.2,31.55)]:guide+=box([a,ya,43.5],[b,yb,55.8])
# Extend the lower guided head; front-to-rear widening prints as a ramp.
guide-=box([-11.3,23.8,39.79],[1.2,31.4,43.91])
for xa,xb in [(-13.45,-9.05),(-1.05,3.35)]:
 guide+=box([xa,31.4,39.8],[xb,34.5,55.8])
for xa,xb in [(-13.45,-11.3),(1.2,3.35)]:guide+=box([xa,30.2,39.8],[xb,31.55,43.6])
guide+=box([-7.3,20,39.8],[-2.8,23.8,43.9])
for xa,xb in [(-13.45,-7.3),(-2.8,3.35)]:guide+=box([xa,20,39.8],[xb,24.3,43.9])
# Flatten only the lower fixed-peg flange tips to meet the shaft's first print layer.
for xa,xb in [(-14.61,-13.79),(3.69,4.51)]:guide-=box([xa,25.5,40.7],[xb,29.7,41.4])
# Preserve straight locking walls, enlarge X play to 0.35 mm/side.
end=solid('Carriage bearing end')
for x in [-8.8,-1.3]:
 end-=box([x-2.3,23.5,35.8],[x+2.3,28.5,39.6])
 end-=(rect(x,26,4.6,5,38.7)+rect(x,26,6,6.4,39.4)).hull()
bolt=solid('Locking bolt')
bolt+=(box([-7,24.5,40.2],[-3.1,24.51,43.91])+box([-11.05,28.6,40.2],[.95,31.2,43.91])).hull()
# Reshape only the final 0.8 mm of the nose; full shank remains above Z37.2.
nose=(rect(-5.05,26,2.3,2.4,36.4)+rect(-5.05,26,3.9,4,37.2)).hull()
bolt=(bolt-box([-8,23,36],[ -2,29,37.2]))+(bolt^nose)
changes={'Locking bolt guide':guide,'Carriage bearing end':end,'Locking bolt':bolt};meshes={}
for name,s in changes.items():
 mm=s.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-8]);assert t.is_watertight and len(t.split())==1,name;t.export(O/(name+'.stl'),file_type='stl_ascii');meshes[name]=t;print(name,t.volume,flush=True)
d=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3);arrays=[];offset=0
for p in d['parts']:
 a=meshes[p['name']].triangles.reshape(-1,3).astype('<f4') if p['name'] in meshes else v[p['offset']//3:p['offset']//3+p['vertices']]
 if p['name']=='Locking bolt guide':p['bed']='bottom'
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arrays.append(a)
d['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(d,separators=(',',':')))
print('BUILT',flush=True)
