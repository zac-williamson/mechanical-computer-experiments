from pathlib import Path
import json,gzip,base64,shutil
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'guide-print-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for p in (R/'neck-candidate').iterdir():
 if p.is_file() and (p.suffix in ['.stl','.json','.md'] or p.name=='Viewer.html') and not (B/p.name).exists():shutil.copy2(p,B/p.name)
for p in B.iterdir():shutil.copy2(p,O/p.name)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def prism(points,y0,y1):return m.Manifold.hull_points([[x,y,z] for x,z in points for y in [y0,y1]])
t=trimesh.load_mesh(B/'Locking bolt guide.stl');old=solid(t);new=old;print("old",old.status(),flush=True)
# 45-degree load-bearing ramps replace unsupported front shoulders.
for xa,xb,root in [(-15.05,-12.25,-12.25),(2.15,4.95,2.15)]:
 tip=xa if root==xb else xb
 new+=prism([(root,43.89),(tip,46.69),(tip,46.75),(root,46.75)],20,25.19);print("ramp",new.status(),flush=True)
 # Small underside step below each anchor root: continuous material to bed.
 a,b=(-13.45,-12.25) if root<0 else (2.15,3.35)
 new+=box([a,24.3,39.8],[b,25,40.51]);print("step",new.status(),flush=True)
 # Remove the 0.1 mm wide residual lip behind the anchor opening.
 new-=box([xa,30,47.199],[xb,30.111,47.211]);print("cut",new.status(),flush=True)
print("pre simplify",new.status(),flush=True)
mm=new.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);print('mesh',new.status(),[(c.volume,c.is_watertight) for c in t.split(only_watertight=False)],flush=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-7]);assert t.is_watertight and len(t.split())==1
t.export(O/'Locking bolt guide.stl',file_type='stl_ascii')
# Preserve exact sliding interface envelope and anchor pegs.
for a,b in [([-10.31,23.79,39.7],[.21,31.21,55.9]),([-14.61,25.19,40.69],[-12.26,30.11,45.01]),([2.16,25.19,40.69],[4.51,30.11,45.01])]:
 assert (((new-old)+(old-new))^box(a,b)).volume()<.001
D=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];off=0
for p in D['parts']:
 a=t.triangles.reshape(-1,3).astype('<f4') if p['name']=='Locking bolt guide' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
print('Added', (new-old).volume(),'removed',(old-new).volume(),flush=True)
for label,mesh in [('before',trimesh.load_mesh(B/'Locking bolt guide.stl')),('after',t)]:
 bad=(mesh.face_normals[:,2]<-.7072)&(mesh.triangles[:,:,2].max(axis=1)>39.801)
 print(label,'steep downward face area',mesh.area_faces[bad].sum(),flush=True)
 for ids in mesh.facets:
  if bad[ids[0]] and mesh.area_faces[ids].sum()>1:print('remaining',mesh.area_faces[ids].sum(),mesh.triangles[ids].min((0,1)),mesh.triangles[ids].max((0,1)),flush=True)
