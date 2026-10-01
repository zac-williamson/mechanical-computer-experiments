from pathlib import Path
import os,json,gzip,base64
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'reliability-candidate';D=json.loads((O/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
def load(n):
 p=next(p for p in D['parts'] if p['name']==n);a=v[p['offset']//3:p['offset']//3+p['vertices']];return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
t=load('L099');print('CLUTCH MESH',t.is_watertight,solid(t).status(),flush=True)
# Rotational envelope of every actual triangle edge at axial stations. Taking
# the maximum norm over each polygonal section bounds the whole cross-section.
tri=t.triangles;edges=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]);xs=sorted(set(np.round(t.vertices[:,0],5)));stations=sorted(set([xs[0],xs[-1]]+[x for a in xs for x in [a-.0001,a+.0001] if xs[0]<x<xs[-1]]));prof=[]
for x in stations:
 lo=edges[:,0,0];hi=edges[:,1,0];den=hi-lo;ok=(np.minimum(lo,hi)<=x+1e-7)&(np.maximum(lo,hi)>=x-1e-7)&(abs(den)>1e-9)
 ee=edges[ok];w=(x-ee[:,0,0])/(ee[:,1,0]-ee[:,0,0]);p=ee[:,0]+w[:,None]*(ee[:,1]-ee[:,0]);r=np.linalg.norm(p[:,1:]-[10.2,0],axis=1)
 if len(r):prof.append([float(x),float(max(r))])
# Polygon uses radius,height coordinates and revolves around Z, then maps Z to X.
poly=[[0,prof[0][0]]]+[[rr,x] for x,rr in prof]+[[0,prof[-1][0]]]
s=m.CrossSection([poly]).revolve(circular_segments=96).rotate([0,90,0]).translate([0,10.2,0]);fork=solid(load('Carriage body'))+solid(load('Carriage bearing end'));hit=fork^s
print('CLUTCH ROTATIONAL ENVELOPE overlap',hit.volume(),'bounds',hit.bounding_box(),flush=True)
# Slightly expanded radial envelope tests 0.15 mm running clearance without
# expanding axial contact flanges that deliberately transfer carriage force.
waist=[(x,r) for x,r in prof if -1.6<=x<=1.6];print('WAIST',waist,flush=True)
bolt=solid(load('Locking bolt'));guide=solid(load('Locking bolt guide'));restraint=[]
for lift in [0,1.9,3.8]:
 bb=bolt.translate([0,0,lift]);pivot=[-5.05,27.6,47.6+lift]
 for axis in [[1,0,0],[0,1,0]]:
  for sign in [-1,1]:
   free=[]
   for deg in [.5,1,2,3,5]:
    M=trimesh.transformations.rotation_matrix(np.deg2rad(sign*deg),axis,pivot);vol=(bb.transform(M[:3])^guide).volume()
    if vol<.02:free.append(deg)
   restraint.append(dict(lift=lift,axis=axis,sign=sign,free_sample_angles=free))
report=dict(clutch_native_watertight=bool(t.is_watertight),clutch_radial_profile=prof,clutch_fork_overlap_mm3=hit.volume(),clutch_overlap_bounds=hit.bounding_box(),bolt_restraint=restraint,limits='Rigid nominal geometry only; no force, wear, compliance or tooth-phase dynamics.')
(O/'Mechanism audit.json').write_text(json.dumps(report,indent=2));print('BOLT RESTRAINT',json.dumps(restraint),flush=True)
