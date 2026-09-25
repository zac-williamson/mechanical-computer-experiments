"""Screen fixture and rod-splice friction pins against other hardware and parts."""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh,manifold3d as m
from wall_pose import joint,transform
from wall_flat_frame import native_envelope
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
frames=[c['frames'][i] for c in json.loads((R/'Compact layout/Compact contact-resolved operation.json').read_text())['cases'] for i in np.linspace(0,len(c['frames'])-1,17,dtype=int)]
fixtures=json.loads((O/'Frame fixture schedule.json').read_text())['fixtures']
hosts={f['part']:{e['part'],e['frame']} for e in fixtures for f in e['fasteners']}
for key in ['clock','write']:
 for i in [1,2]:hosts[key+' rod coupler friction pin '+str(i)]={key+' pinned rod splice bridge',('Control '+key.upper()+' direct rod and pickup') if i==1 else ('bit '+key+' vertical control rod')}
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
scene=[]
for p in P:
 if p['kind']=='elastic':continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']]
 ss=native_envelope(p,a,joint,frames[0]) if p['kind']=='native' else solid(a)
 scene.append((p,ss,np.array(ss.bounding_box()).reshape(2,3)))
pins=[entry for entry in scene if entry[0]['id'] in hosts]
assert len(pins)==sum(len(e['fasteners']) for e in fixtures)+4 and all(p.get('lego_part')=='2780' for p,s,b in pins)
assert not any('M3' in p.get('hardware','') or 'screw' in p['id'].lower() for p in P)
hits={};seen=set();pose_cache={};seen_frames=set();count=0
for f in frames:
 moved={}
 for p,s,b in scene:
  t=transform(p,f)
  if p['kind']=='native':
   c=b.mean(0);d=t[:3,:3]@c+t[:3,3]-c;key=tuple(d)
  else:key=tuple(t.ravel())
  cache_key=(p['id'],key)
  if cache_key not in pose_cache:
   ss=s.translate(d) if p['kind']=='native' else s.transform(t[:3])
   pose_cache[cache_key]=(ss,np.array(ss.bounding_box()).reshape(2,3),key)
  moved[p['id']]=pose_cache[cache_key]
 frame_key=tuple(moved[p['id']][2] for p,s,b in scene)
 if frame_key in seen_frames:continue
 seen_frames.add(frame_key)
 lows=np.array([moved[p['id']][1][0] for p,s,b in scene]);highs=np.array([moved[p['id']][1][1] for p,s,b in scene])
 for p,s,b in pins:
  name=p['id'];ps,pb,pk=moved[name]
  candidates=np.flatnonzero(np.all(np.minimum(pb[1],highs)-np.maximum(pb[0],lows)>1e-5,axis=1))
  for ix in candidates:
   q,s,b=scene[ix];other=q['id']
   if other==name or other in hosts[name]:continue
   qs,qb,qk=moved[other]
   if np.any(np.minimum(pb[1],qb[1])-np.maximum(pb[0],qb[0])<=1e-5):continue
   k=(name,other,pk,qk)
   if k in seen:continue
   seen.add(k);count+=1;v=(ps^qs).volume()
   if v>.005:hits[(name,other)]=max(hits.get((name,other),0),v)
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),pins=len(pins),scope=__doc__,narrow_checks=count,collisions=[dict(pin=a,other=b,volume_mm3=v) for (a,b),v in hits.items()],pin_clearance_pass=not hits,exclusions='Only each pin\'s named mating socket parts; nominal friction interference is intentional. All other printed and native hardware tested across 112 cases, 17 samples each.')
(O/'Frame pin clearance checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
