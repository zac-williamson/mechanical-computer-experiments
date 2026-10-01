from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R0/'neck-candidate')
s=(R0/'validate.py').read_text().split('printed_hits=[]')[0]
s=s.replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
exec(compile(s,'geometry','exec'));by={p['name']:p for p in parts};hits=[]
for f in poses:
 for x in [-11,11]:
  pin=cy(2.5,6.6,30.6,1,[x+f['q'],0,32])
  for p in parts:
   if p['name'] in ['Carriage body','Carriage bearing end','Carriage control rod'] or 'pin' in p['name'] or p['motion'] in ['carriage','lock-band','bolt','lock'] or p['kind']=='elastic':continue
   other=p['s']
   if p['motion']=='rocker':other=other.rotate([0,0,0]) # native pivot/retainer are fixed; lever remote from this height
   b=np.array(pin.bounding_box()).reshape(2,3);c=np.array(other.bounding_box()).reshape(2,3)
   if np.any(b[1]<=c[0]) or np.any(c[1]<=b[0]):continue
   vol=(pin^other).volume()
   if vol>.01:hits.append(dict(q=f['q'],pin_X=x,part=p['name'],overlap_mm3=vol))
# Proposed key-seat space remains inside the existing rod's outside envelope.
keys=[]
for x in [-11,11]:
 k=box([x-4.8,12.4,28.4],[x+4.8,15.11,35.6])-cy(2.5,12.3,15.2,1,[x,0,32])
 keys.append(k)
keyhits=[]
for f in poses:
 for k in keys:
  kk=k.translate([f['q'],0,0]);b=np.array(kk.bounding_box()).reshape(2,3)
  for p in parts:
   if p['name'] in ['Carriage body','Carriage bearing end','Carriage control rod'] or 'attachment pin' in p['name'] or p['kind']=='elastic':continue
   other=p['s'];mo=p['motion']
   if mo in ['carriage','worm','clutch-ring']:other=other.translate([f['q'],0,0])
   if mo=='bolt':other=other.translate([0,0,f['lift']])
   if mo=='lock':other=other.translate([3.75 if f['lift'] else -3.75,0,0])
   if mo=='rocker':
    M=trimesh.transformations.rotation_matrix(math.radians(-f['beta']),[0,0,1],[13.192323604,26.328448698,16]);other=other.transform(M[:3])
   c=np.array(other.bounding_box()).reshape(2,3)
   if np.any(b[1]<=c[0]) or np.any(c[1]<=b[0]):continue
   vol=(kk^other).volume()
   if vol>.01:keyhits.append(dict(q=f['q'],part=p['name'],overlap_mm3=vol))
report=dict(long_pin_hits=hits,key_hits=keyhits,poses=len(poses),key_bounds=[k.bounding_box() for k in keys],limitations='Concept space check only; no geometry published, key print ramps and full assembly/strength still to validate.')
(R0/'rod-tie-study/Space checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
