from band_geometry import ANCHOR_Z
from pathlib import Path
import runpy,os,json,numpy as np,hashlib
import manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'single-band-candidate';os.environ['PLANAR_OUTPUT']=str(O)
# Reuse the published collision representations, without repeating its pose audit.
s=(R/'validate.py').read_text();prefix=s[:s.index('trace=json.loads')].replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)");g={'__name__':'band_access_check'};exec(compile(prefix,'collision representations','exec'),g)
parts=g['parts'];guide=next(p['s'] for p in parts if p['name']=='Locking bolt guide')
def ellipse(y,z):return m.Manifold.cylinder(1,1,circular_segments=64).scale([y,z,1]).rotate([90,0,90]).translate([-.5,0,0])
# Elliptical loop stretched to clear the flattened retaining flange. The
# 1 mm axial width matches the band model; radial thickness is at least 0.6 mm.
ring=ellipse(2.25,2.85)-ellipse(1.65,2.25)
hits=[];steps=[]
for sign in [-1,1]:
 x0=-17 if sign<0 else 7;x1=-13.1 if sign<0 else 3
 path=[(x0,float(y),ANCHOR_Z) for y in np.linspace(17,27.6,17)]+[(float(x),27.6,ANCHOR_Z) for x in np.linspace(x0,x1,17)]
 for n,xyz in enumerate(path):
  band=((ellipse(2,2)-ellipse(1.5,1.5)) if n==len(path)-1 else ring).translate(xyz);bb=np.array(band.bounding_box()).reshape(2,3)
  for p in parts:
   if p['kind']=='elastic':continue
   other=p['s'];mo=p['motion']
   if mo=='bolt':other=other.translate([0,0,3.8])
   elif mo=='lock':other=other.translate([3.75,0,0])
   # Actuator/carriage stay at the centre for installation; their Z bound
   # is below the loop for every permitted X position.
   ob=np.array(other.bounding_box()).reshape(2,3)
   if np.any(bb[1]<=ob[0]+1e-6) or np.any(ob[1]<=bb[0]+1e-6):continue
   vol=(band^other).volume()
   if vol>.01:hits.append(dict(side=sign,step=n,part=p['name'],volume=vol,bounds=(band^other).bounding_box()))
  steps.append(dict(side=sign,centre=xyz))
# Root material must remain intact, rather than creating detached pegs.
old=g['solid'](g['trimesh'].load(R/'band-access/before/Locking bolt guide.stl'))
roots=[]
for sign in [-1,1]:
 edge=-5.05+sign*6;a,b=sorted([edge+sign*.4,edge+sign*1.2]);probe=g['cy'](2.15,a,b,0,[0,27.6,42.8]);lost=(probe-guide).volume();roots.append(dict(side=sign,missing_mm3=lost,passed=lost<.001))
report=dict(passed=not hits and all(p['passed'] for p in roots),poses=len(steps),hits=hits,roots=roots,loop=dict(axial_width=1,inner_Y_radius=1.65,inner_Z_radius=2.25,outer_Y_radius=2.25,outer_Z_radius=2.85),installation='Raise bolt; introduce lower band loop from -Y beside the free peg end, then move inward along X over the retaining flange, then relax the loop onto the shaft after it clears the flange. Assumes a flexible band that can stretch to this loop size.',guide_only_material_removed_mm3=(old-guide).volume(),added_mm3=(guide-old).volume())
(O/'Band insertion checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True);assert report['passed']
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
