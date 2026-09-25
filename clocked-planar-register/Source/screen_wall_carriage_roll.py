"""Exploratory roll-clearance screen, not a constraint or strength certification."""
import sys,json,hashlib
from pathlib import Path
import numpy as np,trimesh,manifold3d as m
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'Source'))
from wall_pose import vertices,example_frames
P=json.loads((root/'Wall register/parts.json').read_text());V=np.load(root/'Wall register/geometry.npz')['vertices'].reshape(-1,3)
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
fixed={mod:sum((solid(V[p['offset']//3:p['offset']//3+p['vertices']]) for p in P if p['kind']=='printed' and p['module']==mod and p.get('motion','fixed')=='fixed'),m.Manifold()) for mod in ['bit','control']}
cases=[('master',['master Carriage fork and roof','master Right carriage bearing support'],'bit',[0,10.2,16],[1,0,0]),('slave',['slave Carriage fork and roof','slave Right carriage bearing support'],'bit',[106,10.2,16],[1,0,0]),('CLOCK actuator',['Control clock Carriage fork and roof','Control clock Right carriage bearing support'],'control',[-132,6.2,-104],[0,0,1]),('WRITE actuator',['Control write Carriage fork and roof','Control WRITE direct rod and pickup'],'control',[-136,-1.8,-184],[0,0,1])]
fs=example_frames();out=[]
for name,names,mod,c,ax in cases:
 for ix in np.linspace(0,len(fs)-1,5,dtype=int):
  ss=m.Manifold()
  for n in names:
   p=next(p for p in P if p['id']==n);a=V[p['offset']//3:p['offset']//3+p['vertices']];ss+=solid(vertices(p,a,fs[ix]))
  row={'carriage':name,'frame':int(ix),'roll_deg_at_0.001_mm3_overlap':{}}
  for sign in [-1,1]:
   def vol(deg):return (ss.transform(trimesh.transformations.rotation_matrix(np.deg2rad(sign*deg),ax,c)[:3])^fixed[mod]).volume()
   hi=next((float(x) for x in np.arange(.1,10.01,.1) if vol(x)>.001),None)
   if hi is not None:
    lo=max(0,hi-.1)
    for _ in range(10):
     mid=(lo+hi)/2
     if vol(mid)>.001:hi=mid
     else:lo=mid
   row['roll_deg_at_0.001_mm3_overlap'][str(sign)]=hi
   if hi is not None:
    contact=ss.transform(trimesh.transformations.rotation_matrix(np.deg2rad(sign*(hi+.05)),ax,c)[:3])^fixed[mod]
    row.setdefault('contact_bounds_after_extra_0.05_deg',{})[str(sign)]=list(contact.bounding_box())
  out.append(row);print(row,flush=True)
(root/'Wall register/Carriage roll screening.json').write_text(json.dumps({'geometry_sha256':hashlib.sha256((root/'Wall register/geometry.npz').read_bytes()).hexdigest(),'scope':'Rigid perturbation about nominal worm axis against all fixed printed parts; five frames of one capture only. Contact can be unintended; no friction, flex, axle clearance or load solution. Excludes gate forks and passive selector.','results':out},indent=2))
