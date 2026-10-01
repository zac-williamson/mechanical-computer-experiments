from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];O=R0/'single-band-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
exec(compile(s,'clearance','exec'))
assert not printed_hits and not native_hits and not band_hits
by={p['name']:p['s'] for p in parts};bolt=by['Locking bolt'];guide=by['Locking bolt guide'];end=by['Carriage bearing end'];body=by['Carriage body'];rows=[]
for q in [-3.75,3.75]:
 for dx in [-.34,0,.34]:assert (bolt^(end+body).translate([q+dx,0,0])).volume()<.02
 for dx in [-.45,.45]:assert (bolt^end.translate([q+dx,0,0])).volume()>.01
 for dy in [-.45,.45]:assert (bolt^end.translate([q,dy,0])).volume()<.02
 # At the mouth, the narrower nose enters with 0.8 mm X/Y offset.
 for dx,dy in [(-.8,0),(.8,0),(0,-.8),(0,.8)]:
  vol=(bolt.translate([0,0,2.6])^end.translate([q+dx,dy,0])).volume();rows.append([q,dx,dy,vol]);assert vol<.02
for lift in np.linspace(0,3.8,39):assert (bolt.translate([0,0,float(lift)])^guide).volume()<.02
for q in np.linspace(-3.75,3.75,31):assert (bolt.translate([0,0,3.8])^(end+body).translate([float(q),0,0])).volume()<.02
# Compare angular constraint against the prior guide at identical pivots.
old_t=trimesh.load_mesh(O/'baseline/Locking bolt guide.stl');oldguide=solid(old_t);oldbolt=solid(trimesh.load_mesh(O/'baseline/Locking bolt.stl'));angles=[]
for lift in [0,1.9,3.8]:
 for axis in [[1,0,0],[0,1,0]]:
  for sign in [-1,1]:
   vals=[]
   for g,btest in [(oldguide,oldbolt),(guide,bolt)]:
    free=0
    for deg in np.arange(.25,8.01,.25):
     M=trimesh.transformations.rotation_matrix(math.radians(sign*deg),axis,[-5.05,27.6,47.6+lift]);v=(btest.translate([0,0,lift]).transform(M[:3])^g).volume()
     if v>.02:break
     free=float(deg)
    vals.append(free)
   angles.append(dict(lift=lift,axis=axis,sign=sign,old_free_deg=vals[0],new_free_deg=vals[1]))
report=dict(passed=True,mouth_offset_checks=rows,angular_checks=angles,straight_engagement_mm=1.5,notes='Straight shank/pocket overlap is 1.5 mm with 0.8 mm nose bevel and 0.7 mm mouth chamfer. No force or print tolerance simulation.')
(O/'Lock improvement checks.json').write_text(json.dumps(report,indent=2));print('LOCK CHECKS',report,flush=True)
