"""Search the actual reduced write core in one plane, scored by total XZ area."""
from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];C=R/'Core layout';O=R/'Investigation';meta=json.loads((C/'printed-parts.json').read_text());hm=json.loads((C/'hardware.json').read_text());v=np.load(C/'hardware.npz')['vertices'];banks={}
for bank in ['Memory','Write']:
 bs=[]
 for p in meta:
  if p['bank']!=bank:continue
  a=trimesh.load(C/(p['id']+'.stl')).vertices.copy()
  if bank=='Write':a=(a-[96.8,0,16])*[-1,1,-1]
  b=np.array([a.min(0),a.max(0)])
  if p['motion']=='carriage':b[0,0]-=4.6;b[1,0]+=4.6
  if p['motion']=='rocker':
   cloud=[]
   for t in np.linspace(-29,29,61):cloud.append(trimesh.transform_points(a,trimesh.transformations.rotation_matrix(np.radians(t),[0,1,0],[13.192323604,10.2,32.128448698])))
   cloud=np.concatenate(cloud);b=np.array([cloud.min(0),cloud.max(0)])
  bs.append(b)
 for h in hm:
  if h.get('bank')!=bank:continue
  a=v[h['offset']//3:h['offset']//3+h['vertices']].copy()
  if bank=='Write':a=(a-[96.8,0,16])*[-1,1,-1]
  b=np.array([a.min(0),a.max(0)])
  if h['motion'] in ['worm','carriage','clutch-ring']:b[0,0]-=4.6;b[1,0]+=4.6
  bs.append(b)
 banks[bank]=np.array(bs)
a=banks['Memory'];results=[];tested=0
for flip in [False,True]:
 b=banks['Write'].copy()
 if flip:b*=np.array([-1,1,-1]);b=np.sort(b,axis=1)
 # All components share Y. No hidden second layer.
 for dz in range(-100,105,2):
  for dx in range(-100,105,2):
   tested+=1;c=b+[dx,0,dz]
   if np.all((a[:,None,0]<c[None,:,1]+.6)&(a[:,None,1]+.6>c[None,:,0]),axis=2).any():continue
   lo=np.minimum(a[:,0].min(0),c[:,0].min(0));hi=np.maximum(a[:,1].max(0),c[:,1].max(0));size=hi-lo
   results.append(dict(flip=flip,offset=[dx,0,dz],size_mm=size.tolist(),xz_area_mm2=float(size[0]*size[2])))
results.sort(key=lambda x:x['xz_area_mm2']);rep=dict(tested=tested,feasible=len(results),best=results[:30],previous_planar_size_mm=[184.8,43.2,119.3],limits=['Core component boxes with carriage and lever sweeps; gear tooth angular sweep not included.','Routing and latch space must be added before claiming register dimensions.','Same-plane only. No claim of global optimum.'])
(O/'Compact planar search.json').write_text(json.dumps(rep,indent=2));print(json.dumps(rep,indent=2))
