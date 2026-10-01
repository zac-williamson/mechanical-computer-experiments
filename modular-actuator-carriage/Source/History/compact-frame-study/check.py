from pathlib import Path
import os,json
import numpy as np
R=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R/'neck-candidate')
src=(R/'validate.py').read_text().split('trace=json.loads')[0]
src=src.replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)")
env={};exec(src,env);parts=env['parts'];cy=env['cy'];box=env['box'];wall=next(p['s'] for p in parts if p['name']=='Left bearing wall')
checks=[]
for z in [8,10,38,40]:
 pin=cy(2.55,24,40,1,[-24.4,0,z])+cy(3.25,31.6,32.4,1,[-24.4,0,z])
 hits=[]
 for p in parts:
  if p['name'] in ['Left bearing wall','Module base'] or p['name'].startswith('Bearing wall pin -24.4'):continue
  vol=(pin^p['s']).volume()
  if vol>.001:hits.append([p['name'],vol])
 # 8 mm-wide wall cannot contain a radius4.5 pad; test 3.8 mm-radius seating volume.
 seat=cy(3.8,24,31.9,1,[-24.4,0,z])
 checks.append(dict(wall_pin_Z=z,hits=hits,seat_missing_mm3=(seat-wall).volume()))
# Test bridge pocket and pins at retained Z rows. Pockets affect the base only;
# bridges occupy Y32.5..40.3 and must clear all other fixed/moving parts.
for x in [-26,35]:
 for z in [0,48]:
  pad=cy(4.5,32.5,40.3,1,[x,0,z]);pin=cy(2.55,32.4,48.4,1,[x,0,z])+cy(3.25,40,40.8,1,[x,0,z])
  hits=[]
  for p in parts:
   if p['name']=='Module base' or p['name'].startswith('Bearing wall pin -24.4'):continue
   vol=((pad+pin)^p['s']).volume()
   if vol>.001:hits.append([p['name'],vol])
  checks.append(dict(frame_pin=[x,z],hits=hits))
report=dict(base_bounds_X=[-31.5,40.5],width=72,current_Y_depth_preserved=True,checks=checks)
(R/'compact-frame-study/checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
