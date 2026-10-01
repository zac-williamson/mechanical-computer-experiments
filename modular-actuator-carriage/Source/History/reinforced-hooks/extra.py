from pathlib import Path
import os,json
from band_geometry import mesh,length,ANCHOR_Z,ROOT_Z
R0=Path(__file__).resolve().parents[1];O=R0/'reinforced-hooks-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().split('trace=json.loads')[0].replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)");exec(compile(s,'load','exec'));by={p['name']:p for p in parts};g=by['Locking bolt guide']['s'];hits=[]
for lift in np.linspace(0,3.8,20):
 b=solid(mesh(float(lift)))
 for p in parts:
  if p['kind']=='elastic':continue
  other=p['s'].translate([0,0,float(lift)]) if p['motion']=='bolt' else p['s']
  v=(b^other).volume()
  if v>.001:hits.append([float(lift),p['name'],v])
print('strict band',hits,flush=True);assert not hits
# A stretched end loop can be lowered beside each open hook and slid over its cap.
ring=box([-.4,-2.4,-3.4],[.4,2.4,3.4])-box([-.5,-1.95,-2.9],[.5,1.95,2.9]);insertion=[]
for x0,x1 in [(-17,-13.1),(7,3)]:
 path=[[x0,27.6,float(z)] for z in np.linspace(60,44.1,20)]+[[float(x),27.6,44.1] for x in np.linspace(x0,x1,16)]
 for xyz in path:
  a=ring.translate(xyz)
  for p in parts:
   if p['kind']=='elastic':continue
   other=p['s'].translate([0,0,3.8]) if p['motion']=='bolt' else p['s']
   v=(a^other).volume()
   if v>.001:insertion.append([xyz,p['name'],v])
print('insertion',insertion[:10],flush=True);assert not insertion
# Vertical insertion of the bolt with the follower hardware fitted.
bolt_hits=[]
for dz in np.linspace(0,24,25):
 for n in ['Locking bolt','Bolt follower axle 3L','Bolt follower front bush','Bolt follower rear bush']:
  v=(g^by[n]['s'].translate([0,0,float(dz)])).volume()
  if v>.001:bolt_hits.append([float(dz),n,v])
assert not bolt_hits,bolt_hits
# Every down-facing surface is <=45 degrees from vertical, apart from bed contact.
t=by['Locking bolt guide']['mesh'];mask=(t.face_normals[:,2]<-.70711)&(t.triangles[:,:,2].max(1)>39.80001);assert t.area_faces[mask].sum()<.001
layer=[]
for h in np.arange(39.8+.16,55.8,.16):
 prev=g.slice(float(h-.1599));sec=g.slice(float(h))
 growth=(sec-prev.offset(.1601, m.JoinType.Round, circular_segments=128)).area()
 if growth>.002:
  print("layer residual",h,growth,(sec-prev.offset(.1601, m.JoinType.Round, circular_segments=128)).to_polygons(),flush=True)
  layer.append([float(h),growth])
assert not layer,layer
report=dict(passed=True,band_lift_samples=20,band_hits=hits,band_loop_length_mm=length(5,0,ANCHOR_Z),hook_root_Z=ROOT_Z,band_insertion_hits=insertion,band_insertion_assumption='Flexible loop stretched to rectangular inner opening 3.9 mm Y by 5.8 mm Z; access from +Z beside the free hook tip then inward along X. Checks current module only, not assembly inside a populated computer.',bolt_insertion_hits=bolt_hits,steep_downward_area_mm2=float(t.area_faces[mask].sum()),layer_height_mm=.16,excess_layer_growth=layer,limits='Geometry only; real band tension, friction and physical print quality not tested.')
(O/'Support-free guide checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
