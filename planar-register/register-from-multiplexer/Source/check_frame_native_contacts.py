from pathlib import Path
import json,numpy as np,trimesh
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';d=json.load(open(O/'Bearing frame datums.json'));names={b['part'] for b in d['bearings']}|{'Unified rear backbone','Detachable bolt guide'};hs=json.load(open(O/'hardware.json'));v=np.load(O/'hardware.npz')['vertices'];hits=[];fits=[]
for n in names:
 t=trimesh.load(O/(n+'.stl'));lo,hi=t.bounds
 for h in hs:
  a=v[h['offset']//3:h['offset']//3+h['vertices']];tri=a.reshape(-1,3,3);pts=np.concatenate([a,tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]);pts=pts[np.all((pts>lo)&(pts<hi),axis=1)]
  if not len(pts):continue
  pts=pts[t.contains(pts)]
  if not len(pts):continue
  dep=float(trimesh.proximity.closest_point(t,pts)[1].max())
  if dep<.03:continue
  row=dict(pair=[n,h['id']],depth_mm=dep)
  if ('pin ' in h['id'].lower()) and dep<.18:fits.append(row)
  else:hits.append(row)
r=dict(unexpected_hits=hits,friction_pin_fits=fits,scope='Native vertices, face centres and edge midpoints against new closed printed solids in the reference pose. Full rotation of X-axis shafts is checked separately.');(O/'Frame native contacts.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));assert not hits
