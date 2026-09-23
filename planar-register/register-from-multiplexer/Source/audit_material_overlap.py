"""Orientation-independent multi-ray occupancy of EVERY broad-phase pair.
Diagnostic finite material samples, not proof for smaller unsampled overlaps.
Uses the displayed poses (gears currently frozen), not the different source-phase poses.
"""
from pathlib import Path
import json,re,base64,gzip,numpy as np,trimesh,itertools,time
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development'
s=(O/'Viewer.html').read_text();D=json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',s,re.S).group(1));v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);P={};rows=[]
for p in D['parts']:
 a=v[p['offset']//3:p['offset']//3+p['vertices']].astype(float);t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);t.merge_vertices(digits_vertex=5);t.update_faces(t.nondegenerate_faces(height=1e-8));t.update_faces(t.unique_faces());P[p['id']]=t

def occupied(mesh,pts):
 # Ray parity ignores inconsistent imported triangle winding. Distinct hit positions
 # eliminate shared-edge and coincident-face duplicates. Two of three rays must agree.
 out=np.zeros(len(pts),dtype=np.int8)
 for d in [[1,.137,.271],[.193,1,.317],[.223,.157,1]]:
  direction=np.tile(np.array(d)/np.linalg.norm(d),(len(pts),1));loc,ids,tri=mesh.ray.intersects_location(pts,direction,multiple_hits=True)
  out+=(np.bincount(ids,minlength=len(pts))%2).astype(np.int8)
 return out>=2
poses=[(3.75,-3.75),(-3.75,-3.75),(3.75,3.75),(-3.75,3.75),(0,0)]
from integrated_cam_math import bolt_lift
seen={}
for qi,(qm,qe) in enumerate(poses):
 shapes={};transforms={}
 for p in D['parts']:
  t=P[p['id']].copy();T=np.eye(4);q=qm if p.get('bank')=='Memory' else qe;mo=p['motion']
  if mo in ['carriage','worm']:T[0,3]=q
  if mo=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
  if mo=='bolt':T[2,3]=bolt_lift(qm,qe)
  if mo=='rocker':
   source=json.loads((O.parent/'register-mux-reference/multiplexer/Switching trace.json').read_text())['frames'];b=min(source,key=lambda f:abs(f['q']-q))['b'];pivot=[13.192323604+(0 if p['bank']=='Memory' else -80.8),10.2,32.128448698+(0 if p['bank']=='Memory' else 16)];T=trimesh.transformations.rotation_matrix(np.radians(b),[0,1,0],pivot)
  if mo=='lock-band':
   # The upper semicircle and straight sides extend with the moving anchor.
   z=t.vertices[:,2];z0=48;z1=69;up=bolt_lift(qm,qe);t.vertices[:,2]+=np.clip((z-z0)/(z1-z0),0,1)*up
  t.apply_transform(T);shapes[p['id']]=t;transforms[p['id']]=T
 for i,(a,b) in enumerate(itertools.combinations(D['parts'],2)):
  na,nb=a['id'],b['id'];ta,tb=shapes[na],shapes[nb];lo=np.maximum(ta.bounds[0],tb.bounds[0]);hi=np.minimum(ta.bounds[1],tb.bounds[1]);ext=hi-lo
  if np.any(ext<.015):continue
  key=(na,nb,tuple(np.round((np.linalg.inv(transforms[na])@transforms[nb]).ravel(),5)),round(bolt_lift(qm,qe),5) if 'lock-band' in [a['motion'],b['motion']] else 0)
  if key in seen:continue
  seen[key]=1
  step=.4;n=np.maximum(1,np.ceil(ext/step).astype(int));axes=[lo[j]+(np.arange(n[j])+.5)*ext[j]/n[j] for j in range(3)];pts=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
  ia=occupied(ta,pts);candidate=pts[ia]
  if not len(candidate):continue
  both=candidate[occupied(tb,candidate)]
  if len(both):
   r=dict(pair=[na,nb],qm=qm,qe=qe,occupied_samples=len(both),estimated_overlap_mm3=float(len(both)*np.prod(ext/n)),sample=both[0].tolist(),sample_bounds=[both.min(0).tolist(),both.max(0).tolist()]);rows.append(r);print(json.dumps(r),flush=True)
 print('POSE DONE',qi,'pairs',len(seen),flush=True)
(O/'Material overlap inventory.json').write_text(json.dumps(dict(method='0.4 mm or finer voxel centres; majority parity from three rays; displayed poses, no group exclusions',poses=poses,overlaps=rows,limits='Finite diagnostic samples; open native surfaces may still produce occupancy ambiguity. Elastic-band deformation is approximated. No automatic intended-contact exemptions.'),indent=2))
