from pathlib import Path
import numpy as np,trimesh,json,gzip,base64
R=Path(__file__).resolve().parent
for name in ['Carriage body','Carriage bearing end']:
 t=trimesh.load(R/'before'/(name+'.stl'));print(name,flush=True)
 for axis,pos in [(2,37),(2,33.5),(0,0),(0,-11.8 if name=='Carriage body' else 11.8),(1,25.8)]:
  o=np.zeros(3);o[axis]=pos;n=np.eye(3)[axis];s=t.section(plane_origin=o,plane_normal=n)
  if s:
   print('SECTION',axis,pos,flush=True)
   for poly in s.discrete:
    # Keep polygon corners; tiny coplanar tessellation is irrelevant.
    p=poly;vec=np.diff(p,axis=0);ids=[i for i in range(len(vec)) if np.linalg.norm(np.cross(vec[i-1],vec[i]))>1e-5]
    print(np.round(p[ids],3).tolist(),flush=True)
D=json.loads((R.parent/'adapted/Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
for p in D['parts']:
 if p['name'] in ['U015','U022','L072','L102','L099','L097','Actuator lever','Locking bolt','Locking bolt guide']:
  a=v[p['offset']//3:p['offset']//3+p['vertices']];print(p['name'],a.min(0),a.max(0),flush=True)
