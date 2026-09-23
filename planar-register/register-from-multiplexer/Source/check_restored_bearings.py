from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';checks=json.load(open(O/'Bearing support checks.json'));names={x['part'] for x in checks['bearings']}|{'Unified rear backbone','Detachable bolt guide'};hs=json.load(open(O/'hardware.json'));v=np.load(O/'hardware.npz')['vertices'];j={p['id']:p for p in json.load(open(O/'Coupled viewer parts.json'))};hits=[];tested=0
for n in names:
 t=trimesh.load(O/(n+'.stl'));s=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 for h in hs:
  p=j[h['id']];joint=p.get('joint')
  if not joint or joint['axis']!='X':continue
  a=v[h['offset']//3:h['offset']//3+h['vertices']];y,z=joint['center'][1:];rad=float(np.linalg.norm(a[:,1:]-[y,z],axis=1).max());lo=float(a[:,0].min());hi=float(a[:,0].max())
  if p['motion'] in ['worm','carriage','clutch-ring']:lo-=4.2;hi+=4.2
  bounds=np.array([[lo,y-rad,z-rad],[hi,y+rad,z+rad]])
  if np.any(bounds[0]>=t.bounds[1]) or np.any(bounds[1]<=t.bounds[0]):continue
  full=m.Manifold.cylinder(hi-lo,rad/np.cos(np.pi/128),circular_segments=128).rotate([0,90,0]).translate([lo,y,z])
  if max(0,(s^full).volume())<.00001:
   tested+=1;continue
  env=m.Manifold()
  tri=a.reshape(-1,3,3);xs=np.unique(a[:,0]);travel=4.2 if p['motion'] in ['worm','carriage','clutch-ring'] else 0
  for l,r in zip(xs[:-1],xs[1:]):
   if r-l<1e-5:continue
   active=tri[(tri[:,:,0].min(1)<r-1e-6)&(tri[:,:,0].max(1)>l+1e-6)]
   if not len(active):continue
   rr=float(np.linalg.norm(active[:,:,1:]-[y,z],axis=2).max())/np.cos(np.pi/128)
   env+=m.Manifold.cylinder(r-l+2*travel,rr,circular_segments=128).rotate([0,90,0]).translate([l-travel,y,z])
  vol=max(0,(s^env).volume());tested+=1
  if vol>.001:hits.append(dict(support=n,hardware=h['id'],envelope_overlap_mm3=vol,overlap_bounds=list((s^env).bounding_box())))
checks['full_rotation_envelope_pairs']=tested;checks['full_rotation_envelope_hits']=hits;checks['scope']='Complete bore circumference plus conservative full-rotation cylindrical hardware envelopes, expanded by 4.2 mm for axial-moving hardware. Does not qualify structural loading or unrelated parts.'
(O/'Bearing support checks.json').write_text(json.dumps(checks,indent=2));print(json.dumps(checks,indent=2));assert not hits
