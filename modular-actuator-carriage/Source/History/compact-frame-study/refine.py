from pathlib import Path
import os,json
import numpy as np
R=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R/'neck-candidate')
src=(R/'validate.py').read_text().split('trace=json.loads')[0]
src=src.replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)")
e={};exec(src,e);parts=e['parts'];cy=e['cy'];box=e['box'];m=e['m']
V=json.loads((R/'neck-candidate/Viewer.html').read_text().split('<script type="application/json" id="data">')[1].split('</script>')[0])
wall=next(p['s'] for p in parts if p['name']=='Left bearing wall')
foot=box([-28.4,24,5.6],[-20.4,32,14.4]);added=foot-wall
probes=[('Left wall foot added material',added,['Left bearing wall','Module base'])]
for z in [10,38]:probes.append(('Left wall pin Z '+str(z),cy(2.55,24,40,1,[-24.4,0,z])+cy(3.25,31.6,32.4,1,[-24.4,0,z]),['Left bearing wall','Module base']))
for x in [-26,35]:
 for z in [0,48]:
  # Same 11 mm span across each X seam. Check inside each tile; the intervening material is outside the footprint.
  pad=cy(4.5,32.5,40.3,1,[x,0,z])
  if z==48:pad=pad^box([-100,32.4,44.2],[100,40.4,60])
  pin=cy(2.55,32.4,48.4,1,[x,0,z])+cy(3.25,40,40.8,1,[x,0,z])
  probes.append((f'Frame attachment {x}, {z}',pad+pin,['Module base']))
hits=[]
for i,tr in enumerate(V['transforms']):
 for name,s,skip in probes:
  ba=np.array(s.bounding_box()).reshape(2,3)
  for p in parts:
   if p['name'] in skip or p['name'].startswith('Bearing wall pin -24.4') or p['kind']=='elastic':continue
   mat=np.array(tr.get(p['name'],np.eye(4).flatten().tolist())).reshape(4,4).T
   b=p['s'].transform(mat[:3]);bb=np.array(b.bounding_box()).reshape(2,3)
   if np.any(ba[1]<=bb[0]+1e-6) or np.any(bb[1]<=ba[0]+1e-6):continue
   vol=(s^b).volume()
   if vol>.001:hits.append(dict(pose=i,probe=name,part=p['name'],mm3=vol))
report=dict(poses=len(V['transforms']),hits=hits,base_X=[-31.5,40.5],new_left_wall_pin_Z=[10,38],frame_pin_X=[-26,35],frame_pin_Z=[0,48],upper_bridge_min_Z=44.2,scope='Proposed pin and bridge-pad envelopes and added wall-foot material against rigid parts at every viewer pose. Not a rebuilt frame or complete assembly-path validation; elastic bands excluded.')
(R/'compact-frame-study/refined-checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
