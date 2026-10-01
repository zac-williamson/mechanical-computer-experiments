from pathlib import Path
import numpy as np,trimesh,json
R=Path(__file__).resolve().parents[1]/'neck-candidate';t=trimesh.load_mesh(R/'Locking bolt guide.stl')
print('mesh',t.is_watertight,len(t.split()),flush=True)
c=t.triangles_center;n=t.face_normals
ids=np.flatnonzero((c[:,0]<-10)&(c[:,0]>-16)&(c[:,1]<32)&(t.area_faces>1e-7))
orig=c[ids]-n[ids]*1e-8
loc,ray,tri=t.ray.intersects_location(orig,-n[ids],multiple_hits=True)
best={}
for p,r,f in zip(loc,ray,tri):
 if f==ids[r]:continue
 d=np.linalg.norm(p-orig[r])
 if d>1e-8 and (r not in best or d<best[r][0]):best[r]=(float(d),int(f))
rows=[]
for r,(dist,f) in sorted(best.items(),key=lambda x:x[1][0]):
 if dist<.65:
  rows.append(dict(thickness=dist,area=float(t.area_faces[ids[r]]),centre=c[ids[r]].tolist(),normal=n[ids[r]].tolist(),opposite=c[f].tolist()))
print(json.dumps(rows[:65],indent=2),flush=True)
Path(__file__).with_name('anchor-audit.json').write_text(json.dumps(rows,indent=2))
