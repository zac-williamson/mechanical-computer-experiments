from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m,itertools
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';meta=json.load(open(O/'printed-parts.json'));new={b['part'] for b in json.load(open(O/'Bearing frame datums.json'))['bearings']}|{'Unified rear backbone','Detachable bolt guide'};shapes={}
for p in meta:
 t=trimesh.load(O/(p['id']+'.stl'));shapes[p['id']]=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
hits=[];tested=0
for p in meta:
 n=p['id'];poses=[shapes[n]]
 if p['motion']=='carriage':poses=[shapes[n].translate([q,0,0]) for q in np.linspace(-3.756,3.756,81)]
 if p['motion']=='bolt':poses=[shapes[n].translate([0,0,q]) for q in np.linspace(0,5.4,55)]
 if p['motion']=='rocker':
  c=np.array([13.192323604+(-85 if p['bank']=='Write' else 0),10.2,32.128448698+(16 if p['bank']=='Write' else 0)]);poses=[shapes[n].translate((-c).tolist()).rotate([0,b,0]).translate(c.tolist()) for b in np.linspace(-30,30,241)]
 for wall in new:
  if n==wall or (n in new and n<wall):continue
  w=shapes[wall];bb=np.array(w.bounding_box()).reshape(2,3)
  for k,a in enumerate(poses):
   ab=np.array(a.bounding_box()).reshape(2,3)
   if np.any(np.minimum(ab[1],bb[1])-np.maximum(ab[0],bb[0])<1e-5):continue
   tested+=1;vol=max(0,(a^w).volume())
   if vol>.001:hits.append(dict(pair=[wall,n],pose=k,overlap_mm3=vol));break
r=dict(checked_fixed_parts=sorted(new),boolean_pairs=tested,hits=hits,scope='Finite independent full carriage travel, bolt lift and lever angle sweeps against every new fixed part. Not a continuous collision proof.');(O/'Frame printed clearance.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));assert not hits
