from pathlib import Path
import json,os,math
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];O=R0/'rod-tie-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().split('printed_hits=[]')[0].replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)").replace('trimesh.Trimesh.Trimesh','trimesh.Trimesh')
exec(compile(s,'load','exec'));by={p['name']:p['s'] for p in parts}
bolt=by['Locking bolt'];end=by['Carriage bearing end'];body=by['Carriage body']
for q in [-3.75,3.75]:
 for dx in [-.34,0,.34]:assert (bolt^(end+body).translate([q+dx,0,0])).volume()<.02
 for dx in [-.45,.45]:assert (bolt^end.translate([q+dx,0,0])).volume()>.01
for q in np.linspace(-3.75,3.75,31):assert (bolt.translate([0,0,3.8])^(end+body).translate([float(q),0,0])).volume()<.02
# Continuous material under the upward-facing bearing contact annuli.
bearing=[]
for n,sign in [('Carriage body',1),('Carriage bearing end',-1)]:
 a,b=sorted([sign*9.101,sign*16.249]);annulus=cy(5.1,a,b,0,[0,10.2,16])-cy(3.46,a-.01,b+.01,0,[0,10.2,16]);v=(annulus-by[n]).volume();assert v<.002;bearing.append(dict(part=n,bearing_support_void_mm3=v,bore_print_axis_vertical=True))
# All added material stays inside the tile and outside neighbours' full bounds.
added=[]
for n in ['Carriage body','Carriage bearing end','Carriage control rod']:
 t=trimesh.load_mesh(O/'baseline'/(n+'.stl'));extra=by[n]-solid(t)
 if extra.volume()>.001:added.append((n,extra))
intervals=[]
for q in [-3.75,3.75]:
 for n,extra in added:
  bb=np.array(extra.translate([q,0,0]).bounding_box()).reshape(2,3)
  for dx,dz in [(80,0),(-80,0),(0,64),(0,-64),(80,64)]:
   for p in parts:
    ob=np.array(p['s'].bounding_box()).reshape(2,3);mo=p['motion']
    if mo in ['carriage','worm','clutch-ring','lock']:ob[0,0]-=3.75;ob[1,0]+=3.75
    if mo=='bolt':ob[1,2]+=3.8
    ob+=np.array([dx,0,dz]);assert np.any(bb[1]<=ob[0]) or np.any(ob[1]<=bb[0]),(n,p['name'],dx,dz)
  intervals.append(dict(part=n,q=q,bounds=bb.tolist()))
report=dict(passed=True,bearing_checks=bearing,locking_endpoints=2,release_samples=31,tile_added_material_bounds=intervals,scope='New material isolated from adjacent modules at X pitch 80 and Z pitch 64; original rod/frame connections retained. Physical stiffness and pin retention untested.')
(O/'Rod tie final checks.json').write_text(json.dumps(report,indent=2));print('FINAL CHECKS PASSED',flush=True)
