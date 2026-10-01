from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];O=R0/'support-free-guide-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
s=s.replace("R.parent/'single-band/band_geometry.py'","R.parent/'support-free-guide/band_geometry.py'")
# Assess changed guide against the full mechanism at every existing pose.
s=s.replace("vol=(sa^sb).volume()","\n   if 'Locking bolt guide' not in names and 'Single lock return band' not in names:continue\n   vol=(sa^sb).volume()")
exec(compile(s,'guide operating checks','exec'));assert not printed_hits and not native_hits and not band_hits
by={p['name']:p for p in parts};guide=by['Locking bolt guide']['s'];old=solid(trimesh.load_mesh(O/'baseline/Locking bolt guide.stl'));added=guide-old
# Neighbours retain identical outer envelope. Explicitly check every added rib.
hits=[]
for dx,dz in [(80,0),(-80,0),(0,64),(0,-64)]:
 for p in parts:
  if p['kind']=='elastic':continue
  for q in [-3.75,3.75]:
   off=[dx+(q if p['motion'] in ['carriage','worm','clutch-ring'] else 0),0,dz]
   if (added^p['s'].translate(off)).volume()>.001:hits.append([dx,dz,p['name']])
assert not hits,hits
report=dict(passed=True,poses=len(poses),neighbour_intersections=hits,added_mm3=added.volume(),removed_mm3=(old-guide).volume(),limits='Geometric clearance checks; band is idealized. No physical print or force test.')
(O/'Guide print redesign checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
