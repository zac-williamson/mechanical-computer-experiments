"""Conservative component-envelope search, not complete register packaging.
All original parts retained. No collisions are hidden by omitting structural parts.
A feasible row establishes separation of two cores, NOT routes or latch integration.
"""
from pathlib import Path
import json,numpy as np,trimesh,itertools
R=Path(__file__).resolve().parents[1];B=R.parent/'work/register-mux-reference/multiplexer';O=R/'Investigation'
meta=json.loads((B/'printed-parts.json').read_text());hm=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];par=json.loads((B/'parameters.json').read_text());pv=np.array(par['pivot'])
bounds=[];names=[]
for p in meta:
 t=trimesh.load(B/(p['id']+'.stl'));b=t.bounds.copy()
 if p['motion']=='carriage':b[0,0]-=4.6;b[1,0]+=4.6
 if p['motion']=='rocker':
  cloud=[]
  for deg in np.arange(-29,29.01,.25):
   T=trimesh.transformations.rotation_matrix(np.radians(deg),[0,1,0],pv);cloud.append(trimesh.transform_points(t.vertices,T))
  cloud=np.concatenate(cloud);b=np.array([cloud.min(0)-.02,cloud.max(0)+.02])
 bounds.append(b);names.append(p['id'])
for p in hm:
 a=v[p['offset']//3:p['offset']//3+p['vertices']];b=np.array([a.min(0),a.max(0)]);mo=p.get('motion')
 if mo not in ['fixed','carriage']:
  axis=1 if mo=='gear' else 0
  centre=np.array([0.,10.2,24 if axis==1 else 16 if mo in ['worm','input'] else 0])
  if mo in ['A','B'] and p['id'] not in ['L072','L102']:centre[2]=-18.4
  if 'idler' in mo:centre[1:]=[10.2+np.sqrt(144-9.2**2),-9.2]
  ij=[j for j in range(3) if j!=axis];radius=np.linalg.norm(a[:,ij]-centre[ij],axis=1).max();b[0,ij]=centre[ij]-radius;b[1,ij]=centre[ij]+radius
 if mo in ['carriage','worm','clutch-ring']:b[0,0]-=4.6;b[1,0]+=4.6
 bounds.append(b);names.append(p['id'])
# Conservative elastic band box from anchor plus maximum 9.2mm lever radius, both sides plus band allowance.
bounds.append(np.array([[0,7.5,17],[42,12.9,45.5]]));names.append('conservative band envelope')
a=np.array(bounds);lo=a[:,0].min(0);hi=a[:,1].max(0);res=[];count=0
for flip in [False,True]:
 b=a.copy()
 if flip:
  b[:,:,0]*=-1;b[:,:,2]*=-1;b=np.sort(b,axis=1)
 for dy in [0,8,16,24,32,40,48]:
  for dz in np.arange(0,104.01,2):
   # At each Y/Z separation find smallest nonnegative X translation in sampled grid.
   for dx in np.arange(0,100.01,2):
    count+=1;c=b+np.array([dx,dy,dz])
    hit=np.all((a[:,None,0,:] < c[None,:,1,:]+.6)&(a[:,None,1,:]+.6 > c[None,:,0,:]),axis=2)
    if not hit.any():
     l=np.minimum(lo,c[:,0].min(0));u=np.maximum(hi,c[:,1].max(0));s=u-l
     # Fractional overlap of the complete XZ envelope as an explicit visibility proxy.
     overlap=np.prod(np.maximum(0,np.minimum(hi[[0,2]],c[:,1].max(0)[[0,2]])-np.maximum(lo[[0,2]],c[:,0].min(0)[[0,2]])))
     vis=overlap/np.prod((hi-lo)[[0,2]])
     res.append(dict(flip_Y_180=flip,offset=[float(dx),dy,float(dz)],size=s.tolist(),volume_mm3=float(np.prod(s)),xz_envelope_overlap_fraction=float(vis)))
     break
res.sort(key=lambda x:x['volume_mm3']);visible=[x for x in res if x['xz_envelope_overlap_fraction']<.1]
r={'search_count':count,'grid_mm':2,'component_gap_mm':.6,'core_swept_envelope_mm':(hi-lo).tolist(),'feasible_layouts':len(res),'best_volume':res[:10],'best_low_projection_overlap':visible[:10],'limits':['Component AABBs and rotation envelopes are conservative; a rejected row is not proof of actual collision.','No feedback gearing, data routes, coupling connectors, inter-core mounts or latch included.','Envelope overlap is only a visibility proxy, not a rendering/occlusion proof.','Results are within this two-core family and grid, not a global minimum for a register.']}
(O/'Packing search.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
