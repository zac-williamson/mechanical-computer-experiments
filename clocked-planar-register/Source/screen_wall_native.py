"""Full-revolution native envelopes against printed solids at operation poses.

An envelope hit is a potential collision, not proof: working tooth/lever and
ring/fork contacts need phase-resolved narrow checks. No hit proves separation
for every rotation phase at the sampled relative poses, not between samples.
"""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh,manifold3d as m
from wall_pose import joint,transform,example_frames
R=Path(__file__).resolve().parents[1]/'Wall register';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
parts=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
printed=[];native=[]
cases=json.loads((R.parent/'Compact layout/Compact contact-resolved operation.json').read_text())['cases']
frames=[case['frames'][i] for case in cases for i in np.linspace(0,len(case['frames'])-1,17,dtype=int)]

def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
for p in parts:
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 if p['kind']=='printed':printed.append((p,solid(a)))
 elif p['kind']=='native' and p.get('lego_part')!='2780' and 'pin' not in p['id'].lower():
  sh,ax,cen,ang=joint(p,frames[0]);axis=int(np.argmax(abs(ax)))
  if p.get('axis') is not None:axis=p['axis']
  elif p.get('motion')=='gear' or 'stop-axle' in p['id'] or 'retainer' in p['id']:axis=1
  cross=[j for j in range(3) if j!=axis];centre=(a.min(0)+a.max(0))/2
  # Slice actual triangle edges on BOTH sides of every axial step. Merely
  # joining maximum-radius vertices creates fictitious cones at gear hubs
  # and axle stops, falsely reporting bearing-wall collisions.
  tri=a.reshape(-1,3,3);edges=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]);dx=edges[:,1,axis]-edges[:,0,axis]
  edges=edges[abs(dx)>1e-9];dx=edges[:,1,axis]-edges[:,0,axis]
  xx=np.unique(np.round(a[:,axis],4));eps=1e-5
  grid=np.unique(np.clip(np.r_[xx-eps,xx+eps,(xx[:-1]+xx[1:])/2],a[:,axis].min()+1e-7,a[:,axis].max()-1e-7))
  radii=[];positions=[]
  for position in grid:
   frac=(position-edges[:,0,axis])/dx;ok=(frac>=0)&(frac<=1)
   if not ok.any():continue
   pts=edges[ok,0]+frac[ok,None]*(edges[ok,1]-edges[ok,0])
   radii.append(np.linalg.norm(pts[:,cross]-centre[cross],axis=1).max());positions.append(position)
  profile=np.vstack([[0,positions[0]],np.stack([radii,positions],axis=1),[0,positions[-1]]])
  mesh=trimesh.creation.revolve(profile,sections=64)
  mesh.apply_transform(trimesh.geometry.align_vectors([0,0,1],np.eye(3)[axis]));delta=centre.copy();delta[axis]=0;mesh.apply_translation(delta)
  s=m.Manifold(m.Mesh64(mesh.vertices.astype(float),mesh.faces.astype(np.uint64)))
  assert s.status()==m.Error.NoError,p['id']
  native.append((p,s))
hits={};cache={};seen=set();checks=0;count=0
# A coarse frame stride is only for the first screening run; report it explicitly.
for fi,f in enumerate(frames):
 matrices=[transform(p,f)[:3] for p,s in printed]
 # Remove rotation about each native part's own symmetry axis by expressing
 # only its transformed centre translation. Amplifier followers translate on arcs.
 nm=[]
 for p,s in native:
  t=transform(p,f);c=(np.array(s.bounding_box())[:3]+np.array(s.bounding_box())[3:])/2
  d=trimesh.transform_points(c[None,:],t)[0]-c;nm.append(d)
 key=tuple(np.round(np.concatenate([t.ravel() for t in matrices]+nm),5))
 if key in seen:continue
 seen.add(key);count+=1
 pp=[(p,s.transform(t),tuple(np.round(t.ravel(),5))) for (p,s),t in zip(printed,matrices)]
 for (p,s),d in zip(native,nm):
  ss=s.translate(d);b=np.array(ss.bounding_box()).reshape(2,3)
  for q,t,qt in pp:
   bb=np.array(t.bounding_box()).reshape(2,3)
   if np.any(np.minimum(b[1],bb[1])-np.maximum(b[0],bb[0])<=0):continue
   k=(p['id'],q['id'],tuple(np.round(d,5)),qt)
   if k not in cache:cache[k]=(ss^t).volume();checks+=1
   vol=cache[k]
   if vol>.001:
    pair=(p['id'],q['id'])
    if pair not in hits or hits[pair]['volume_mm3']<vol:hits[pair]=dict(native=pair[0],printed=pair[1],volume_mm3=vol,sampled_frame=fi)
result=dict(scope=__doc__,geometry_sha256=digest,frame_stride=1,case_coverage='112 inherited cases with 17 samples each',unique_poses=count,narrow_checks=checks,potential_collisions=sorted(hits.values(),key=lambda h:-h['volume_mm3']),mechanically_qualified=False)
(R/'Rotating envelope screening.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
