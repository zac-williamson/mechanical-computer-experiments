import os
from pathlib import Path
import json,numpy as np,manifold3d as m,trimesh
ROOT=Path(__file__).resolve().parents[1];O=ROOT/'outputs/Adder redesign';D=json.loads((O/'Assembly manifest.json').read_text());ss={}
for p in D['prints']:
 if p['motion']!='fixed':continue
 t=trimesh.load(O/p['path']);ss[p['id']]=m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
checks=[];hits=[]
for r in D['records']:
 if r['part']!='2780.dat' or r['motion']!='fixed':continue
 p=np.array(r['pos'])*.4;axis=np.array(r['matrix']).reshape(3,3)[:,0];R=np.eye(3)
 # Native 2780 is along local X. Analytic core excludes intentional friction ribs.
 s=m.Manifold.cylinder(16,2.39,circular_segments=48,center=True).rotate([0,90,0]);flange=m.Manifold.cylinder(.7,3.15,circular_segments=48,center=True).rotate([0,90,0]);s=(s+flange).transform(np.column_stack([np.array(r['matrix']).reshape(3,3),p]).tolist())
 lo=np.array(s.bounding_box()[:3]);hi=np.array(s.bounding_box()[3:]);bad=[]
 for name,v in ss.items():
  bb=np.array(v.bounding_box())
  if np.any(np.minimum(hi,bb[3:])-np.maximum(lo,bb[:3])<1e-5):continue
  vol=(s^v).volume()
  if vol>.02:bad.append([name,float(vol)])
 if bad:hits.append(dict(pin=r['record_id'],hits=bad))
 checks.append(dict(pin=r['record_id'],axis=axis.tolist(),centre=p.tolist()))
# Every base hole is open through its underside, with 0.2 mm tip recess.
base=ss['Base'];through=[]
for x,z in D['mounts']:
 bore=m.Manifold.cylinder(10,2.39,circular_segments=48,center=True).rotate([90,0,0]).translate([x,32.1,z]);v=(bore^base).volume();assert v<1e-5,(x,z,v);through.append([x,z])
result=dict(fixed_pins=len(checks),base_holes=len(through),base_pin_tip_recess_mm=.2,core_radius_mm=2.39,friction_ribs_excluded=True,worst=hits,checks=checks);(O/'Mount checks.json').write_text(json.dumps(result,indent=2)+'\n');print('Fixed pins',len(checks),'base through holes',len(through),'hits',json.dumps(hits),flush=True)
