from pathlib import Path
import runpy,os,json,numpy as np,hashlib
import manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'band-candidate';os.environ['PLANAR_OUTPUT']=str(O)
# Reuse the published collision representations, without repeating its pose audit.
s=(R/'validate.py').read_text();prefix=s[:s.index('trace=json.loads')];g={'__name__':'band_access_check'};exec(compile(prefix,'collision representations','exec'),g)
parts=g['parts'];guide=next(p['s'] for p in parts if p['name']=='Locking bolt guide')
def ellipse(y,z):return m.Manifold.cylinder(1,1,circular_segments=64).scale([y,z,1]).rotate([90,0,90]).translate([-.5,0,0])
# Elliptical loop stretched to clear the flattened retaining flange. The
# 1 mm axial width matches the band model; radial thickness is at least 0.6 mm.
ring=ellipse(2.25,2.85)-ellipse(1.65,2.25)
hits=[];steps=[]
for sign in [-1,1]:
 x0=-17 if sign<0 else 7;x1=-13.1 if sign<0 else 3
 path=[(x0,float(y),42.8) for y in np.linspace(17,27.6,17)]+[(float(x),27.6,42.8) for x in np.linspace(x0,x1,17)]
 for n,xyz in enumerate(path):
  band=ring.translate(xyz);bb=np.array(band.bounding_box()).reshape(2,3)
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
   if vol>.01:hits.append(dict(side=sign,step=n,part=p['name'],volume=vol))
  steps.append(dict(side=sign,centre=xyz))
# Root material must remain intact, rather than creating detached pegs.
old=g['solid'](g['trimesh'].load(R/'band-access/before/Locking bolt guide.stl'))
roots=[]
for sign in [-1,1]:
 edge=-5.05+sign*6;a,b=sorted([edge+sign*.4,edge+sign*1.2]);probe=g['cy'](2.15,a,b,0,[0,27.6,42.8]);lost=(probe-guide).volume();roots.append(dict(side=sign,missing_mm3=lost,passed=lost<.001))
report=dict(passed=not hits and all(p['passed'] for p in roots),poses=len(steps),hits=hits,roots=roots,loop=dict(axial_width=1,inner_Y_radius=1.65,inner_Z_radius=2.25,outer_Y_radius=2.25,outer_Z_radius=2.85),installation='Raise bolt; introduce lower band loop from -Y beside the free peg end, then move inward along X over the retaining flange. Assumes a flexible band that can stretch to this loop size.',guide_only_material_removed_mm3=(old-guide).volume(),added_mm3=(guide-old).volume())
(O/'Band insertion checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True);assert report['passed']
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
# Actual-mesh inspection images, isolated to make the anchor openings visible.
s=(R/'render_review.py').read_text().replace("idx=4;tri=[];cols=[]", "D['parts']=[p for p in D['parts'] if p['name']=='Locking bolt guide']\nidx=4;tri=[];cols=[]").replace("'Review views.png'","'Band anchor views.png'").replace('Work in progress · bolt released · X: rods/axles; Z: rows; +Y: rear/base','Fixed band anchors · enter from -Y, then slip inward along X over each free end')
exec(compile(s,'anchor render','exec'),{'__name__':'__main__','__file__':str(R/'render_review.py')})
(O/'Checked mesh hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.stl')},indent=2))

# Print the broad rear face on the bed: pin holes vertical, peg shaft and
# flattened flange start on the same Y layer rather than as separate islands.
sg=guide.rotate([-90,0,0]);bb=sg.bounding_box();sg=sg.translate([-bb[0],-bb[1],-bb[2]]);layers=[]
for h in np.arange(.3,sg.bounding_box()[5],.2):
 prev=sg.slice(h-.2);now=sg.slice(h);islands=[c.area() for c in now.decompose() if c.area()>.1 and (c^prev.offset(.21)).area()<.01]
 if islands:layers.append(dict(height=float(h),islands=islands))
(O/'Band guide print checks.json').write_text(json.dumps(dict(bed_face='+Y rear face',layer_mm=.2,detached_islands=layers,passed=not layers,limitation='Layer connectivity screen only; not a guarantee of surface finish or strength.'),indent=2))
assert not layers,layers
print('Rear-face print orientation: no detached islands in 0.2 mm layer screen.',flush=True)

t=next(p['mesh'].copy() for p in parts if p['name']=='Locking bolt guide')
t.apply_transform(g['trimesh'].geometry.align_vectors([0,1,0],[0,0,-1]));t.apply_translation(-t.bounds[0]);assert t.is_watertight
t.export(O/'Locking bolt guide print.stl')
(O/'Checked mesh hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.stl')},indent=2))
