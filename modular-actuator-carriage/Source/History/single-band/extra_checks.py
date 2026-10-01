from pathlib import Path
import os,json
from band_geometry import mesh,length,ANCHOR_Z
R0=Path(__file__).resolve().parents[1];O=R0/'single-band-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().split('trace=json.loads')[0].replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)");exec(compile(s,'load','exec'));by={p['name']:p for p in parts};hits=[];maxband=0
for lift in np.linspace(0,3.8,20):
 b=solid(mesh(float(lift)))
 for p in parts:
  if p['kind']=='elastic':continue
  other=p['s'].translate([0,0,float(lift)]) if p['motion']=='bolt' else p['s'];v=(b^other).volume();maxband=max(maxband,v)
  if v>.001:hits.append([float(lift),p['name'],v])
print('BAND CONTACT',maxband,hits,flush=True);assert not hits
old=solid(trimesh.load_mesh(O/'baseline/Locking bolt.stl'));new=by['Locking bolt']['s']
for name,region in [('locking tip',box([-8,23,36],[-2,29,39.8])),('axle bore surround',cy(3.49,23.9,31.3,1,[-5.05,0,47.6]))]:
 v=(((new-old)+(old-new))^region).volume();assert v<.001,(name,v)
# Check changed parts and the band against the neighbouring module at both travel ends.
nh=[]
for lift in [0,3.8]:
 subjects=[by['Locking bolt guide']['s'],by['Locking bolt']['s'].translate([0,0,lift]),solid(mesh(lift))]
 for dx,dz in [(80,0),(-80,0),(0,64),(0,-64)]:
  for q in [-3.75,3.75]:
   for p in parts:
    if p['kind']=='elastic':continue
    shift=[dx+(q if p['motion'] in ['carriage','worm','clutch-ring'] else 0),0,dz+(lift if p['motion']=='bolt' else 0)];other=p['s'].translate(shift);bb=np.array(other.bounding_box()).reshape(2,3)
    for a in subjects:
     aa=np.array(a.bounding_box()).reshape(2,3)
     if np.any(aa[1]<=bb[0]) or np.any(bb[1]<=aa[0]):continue
     v=(a^other).volume()
     if v>.01:nh.append([dx,dz,q,p['name'],v])
assert not nh,nh
report=dict(passed=True,maximum_band_intersection_mm3=maxband,band_samples=20,tile_neighbour_hits=nh,old_loop_mm=length(6),new_loop_mm=length(5,0,ANCHOR_Z),loop_reduction_mm=length(6)-length(5,0,ANCHOR_Z),band_model_diameter_mm=.5,limits='Idealised band path; real band cross-section, stiffness, friction and stretch not supplied.')
(O/'Single band checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
