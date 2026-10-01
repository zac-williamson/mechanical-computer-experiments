import os
from pathlib import Path
import json,gzip,base64,math
import numpy as np,trimesh,manifold3d as m
R=Path(os.environ['PLANAR_OUTPUT']) if 'PLANAR_OUTPUT' in os.environ else Path(__file__).resolve().parent/'build'
ROOT=R.parents[2]
D=json.loads((R/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=40)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
import runpy
clutch_envelope=runpy.run_path(str(R.parent/'native_envelopes.py'))['clutch_envelope']
parts=[];quality=[]
for p in D['parts']:
 a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p);p['mesh']=t;p['s']=solid(t)
 if p['kind']=='printed':
  quality.append(dict(part=p['name'],watertight=t.is_watertight,solids=len([c for c in p['s'].decompose() if c.volume()>1e-5])))
 else:
  axis=p.get('axis');name=p['name'];bb=t.bounds;c=bb.mean(0)
  if p['kind']=='elastic':p['s']=p['s'].hull()
  elif 'pin ' in name or 'joining pin' in name:
   p['s']=cy(2.55,bb[0,axis],bb[1,axis],axis,c)+cy(3.25,c[axis]-.4,c[axis]+.4,axis,c)
  elif name in ['reaction-stop-axle','pivot-stop-axle']:
   # The stop is wider than the cross-axle shaft. A full-length stop-radius
   # cylinder falsely rejects clear space beside the actual 2.4 mm shaft.
   p['s']=cy(3.21,float(bb[0,2]),7.6001,2,c)+cy(2.41,7.6,float(bb[1,2]),2,c)
  elif name=='L099':p['s']=clutch_envelope(t)
  elif p['s'].status()!=m.Error.NoError:
   if axis is None:axis=2 if name in ['U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer'] else 0
   yz=[i for i in range(3) if i!=axis];r=float(np.linalg.norm(t.vertices[:,yz]-c[yz],axis=1).max())
   p['s']=cy(r,bb[0,axis],bb[1,axis],axis,c)
 parts.append(p)
trace=json.loads((R.parent/'Switching trace.json').read_text())['frames']
# Use source-linked lever poses for the carriage traversal.
poses=[]
for q,l in [(-3.75,0),(-3.75,1.9),(-3.75,3.8)]+[(float(q),3.8) for q in np.linspace(-3.75,3.75,9)]+[(3.75,1.9),(3.75,0)]:
 f=min(trace,key=lambda f:abs(f['q']-q));poses.append(dict(q=q,lift=l,beta=f['b']))
moving=[f for f in trace if f.get('moving') and abs(f['q'])<=3.75]
if moving:
 for i in np.linspace(0,len(moving)-1,12,dtype=int):
  f=moving[i];poses.append(dict(q=f['q'],lift=3.8,beta=f['b']))
def loop(a,b,inner,outer,z0,z1):
 def cap(r):return (cy(r,z0,z1,2,[a[0],a[1],0])+cy(r,z0,z1,2,[b[0],b[1],0])).hull()
 return cap(outer)-cap(inner)
printed_hits=[];native_hits=[];band_hits=[]
for f in poses:
 posed=[]
 for p in parts:
  s=p['s'];mo=p['motion'];name=p['name'];M=np.eye(4)
  if mo=='rocker':M=trimesh.transformations.rotation_matrix(math.radians(-f['beta']),[0,0,1],[13.192323604,26.328448698,16])
  if mo in ['carriage','worm','clutch-ring']:M[0,3]+=f['q']
  if mo=='bolt':M[2,3]+=f['lift']
  if mo=='lock':
   rad=12.;lo=0.;hi=8.25
   for _ in range(60):
    cx=(lo+hi)/2
    if math.sqrt(144-(3.75-cx)**2)-math.sqrt(144-(-3.75-cx)**2)<3.8:lo=cx
    else:hi=cx
   cx=(lo+hi)/2;cz=47.6+math.sqrt(144-(3.75-cx)**2)
   M[0,3]=math.sqrt(144-(cz-47.6-f['lift'])**2)-cx
  # Bands are checked independently below; a rigid translation would be wrong.
  if p['kind']=='elastic':
   if mo=='single-lock-band':
    band_mesh=runpy.run_path(str(R.parent/D.get('lock_band_geometry','single-band/band_geometry.py')))['mesh']
    s=solid(band_mesh(f['lift']))
   elif mo=='lock-band':
    x=-13.1 if name.startswith('Left') else 3.0
    s=loop([27.6,42.8],[27.6,49.6+f['lift']],1.5,2.1,x-.5,x+.5).rotate([90,0,90])
   else:
    rot=trimesh.transformations.rotation_matrix(math.radians(-f['beta']),[0,0,1],[13.192323604,26.328448698,16])
    anchor=trimesh.transform_points([[22.192323604,24.828448698,16]],rot)[0]
    s=loop([36.592323604,22.328448698],anchor[:2],2,3.2,15.4,16.6)
  s=s.transform(M[:3]);posed.append((p,s,np.array(s.bounding_box()).reshape(2,3)))
 for i,(a,sa,ba) in enumerate(posed):
  for b,sb,bb in posed[i+1:]:
   if a['kind']!='printed' and b['kind']!='printed':continue
   if np.any(ba[1]<=bb[0]+1e-6) or np.any(bb[1]<=ba[0]+1e-6):continue
   if a['name'] in b.get('mates',[]) or b['name'] in a.get('mates',[]):continue
   # Retained functional contact surfaces are inherited from the source.
   names={a['name'],b['name']}
   if 'Actuator return band' in names and any(n in names for n in ['Actuator lever','Lower actuator cheek']):continue
   # Retained lever/carriage geometry is checked too.
   if names=={'U022','Actuator lever'}:continue # Intended tooth contact; cylinder fills tooth spaces.
   # The actual clutch rotational profile is checked against the fork.
   vol=(sa^sb).volume()
   if vol>.04:
    hit=dict(**f,a=a['name'],b=b['name'],volume=vol,bounds=(sa^sb).bounding_box())
    (band_hits if 'elastic' in [a['kind'],b['kind']] else printed_hits if a['kind']==b['kind']=='printed' else native_hits).append(hit)
report=dict(quality=quality,printed_interferences=printed_hits,hardware_interferences=native_hits,band_interferences=band_hits,poses=len(poses),notes=['Native open meshes use conservative solid envelopes. Pin friction fits are excluded only for named mating parts.','Clutch/fork uses the actual full-rotation profile. Gear/lever tooth engagement remains an intentional contact, not a force-driven validation.','Band loops checked at each pose; actual tension and deformation require physical testing.'])
(R/'Clearance checks.json').write_text(json.dumps(report,indent=2))
print('QUALITY',quality,flush=True)
for label,hits in [('PRINTED',printed_hits),('HARDWARE',native_hits),('BANDS',band_hits)]:
 pairs={}
 for h in hits:
  key=(h['a'],h['b'])
  if key not in pairs or pairs[key]['volume']<h['volume']:pairs[key]=h
 print(label,json.dumps(list(pairs.values()),indent=2),flush=True)
