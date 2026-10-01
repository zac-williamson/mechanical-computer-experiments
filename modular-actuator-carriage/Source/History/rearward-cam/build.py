from pathlib import Path
import shutil,json,gzip,base64
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'rearward-cam-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for p in A.iterdir():
 if p.is_file() and p.suffix in ['.stl','.json','.md','.html']:
  if not (B/p.name).exists():shutil.copy2(p,B/p.name)
  shutil.copy2(B/p.name,O/p.name)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
old=solid(trimesh.load_mesh(B/'Lock control rod.stl'))
section=old^box([-12.5,10,39.9],[12.5,10.1,54.1])
extra=section.translate([0,-10,0]).scale([1,55,1]).translate([0,14.1,0]);new=(old+extra).simplify(.00001)
mm=new.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t=trimesh.util.concatenate([c for c in t.split(only_watertight=False) if abs(c.volume)>1e-8]);assert t.is_watertight and len(t.split())==1;t.export(O/'Lock control rod.stl',file_type='stl_ascii')
D=json.loads((B/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];off=0
for p in D['parts']:
 a=v[p['offset']//3:p['offset']//3+p['vertices']].copy()
 if p['name']=='Lock control rod':a=t.triangles.reshape(-1,3)
 if p['name']=='Bolt follower front bush':a[:,1]+=4
 p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
(O/'Cam extension changes.json').write_text(json.dumps(dict(added_mm3=(new-old).volume(),extension_X=[-12.5,12.5],old_max_Y=14.2,new_max_Y=19.6,guide_front_Y=20,guide_gap_mm=.4,follower_old_Y=[11.6,15.6],follower_new_Y=[15.6,19.6],cam_profile='Unchanged XZ section',bed_Z=40),indent=2));print('Built extended cam and moved follower +4Y',flush=True)
