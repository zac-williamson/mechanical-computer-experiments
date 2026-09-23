"""Y capture regression: actual lips, nominal stroke, off-axis poses, and play bound."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';g=trimesh.load(O/'Detachable bolt guide.stl');b=trimesh.load(O/'Direct lock bolt.stl');BX=-5.05
# Verify solid retaining lands on BOTH sides of the broad head, not just
# non-intersection of a prescribed animation. The back lands were missing.
points=np.array([[BX+side*x,y,z] for side in [-1,1] for x in np.linspace(4.65,5.95,8) for y in [17.5,25.7] for z in np.linspace(61.05,69.95,40)])
assert g.contains(points).all(),'Missing front/rear retaining material'
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices),np.array(t.faces,dtype=np.uint64)))
sg,sb=solid(g),solid(b);c=np.array([BX,20,48.7]);nominal=[];blocked=[]
for lift in np.linspace(0,5.4,55):
 a=sb.translate([0,0,float(lift)]);nominal.append(max(0,(a^sg).volume()))
 for angle in [-5,5]:
  tilted=a.translate((-c).tolist()).rotate([-angle,0,0]).translate(c.tolist());volume=max(0,(tilted^sg).volume());assert volume>.01,(lift,angle,volume);blocked.append(volume)
assert max(nominal)<.001
# Cross-sections at Z61 and Z70 always pass through the broad head, and Z49
# through its straight tip, for lifts 0..5.4 and pitch -5..+5 degrees.
# At each section the rotated slab occupies y0+(Z-48.7)*tan(theta)+dy.
# Intersect all allowable dy intervals: this permits Y translation while
# tilting, instead of incorrectly assuming the bolt pivots about a fixed point.
def allowed(angle):
 a=np.deg2rad(angle);cs=np.cos(a);tt=np.tan(a);lower=[];upper=[]
 for z,part_lo,part_hi,guide_lo,guide_hi in [(61,18,25.2,17.6,25.6),(70,18,25.2,17.6,25.6),(49,18,22,17.7,22.3)]:
  ylo=20+(part_lo-20)/cs+(z-48.7)*tt;yhi=20+(part_hi-20)/cs+(z-48.7)*tt
  lower.append(guide_lo-ylo);upper.append(guide_hi-yhi)
 return max(lower),min(upper)
bounds=[]
for sign in [-1,1]:
 assert allowed(sign*5)[0]>allowed(sign*5)[1]
 lo,hi=0.,5.
 for _ in range(40):
  mid=(lo+hi)/2;a,b=allowed(sign*mid)
  if a<=b:lo=mid
  else:hi=mid
 bounds.append(sign*hi)
r=dict(retaining_faces_verified=True,full_height_rear_retaining_land_mm=9.0,head_Y_clearance_each_side_mm=.4,minimum_lip_overlap_with_maximum_X_translation_mm=1.0,stroke_mm=[0,5.4],stroke_samples=55,maximum_nominal_overlap_mm3=max(nominal),minimum_interference_at_five_degree_pitch_mm3=min(blocked),pitch_outer_bounds_degrees=bounds,allows_Y_translation_in_pitch_bound=True,scope='Rigid geometry only. Analytic pitch bounds use opposed head lands at Z61/70 and straight tip at Z49 across the full stroke. Contact prevents a five-degree tilt even allowing Y translation. Does not establish elastic stiffness, wear, or combined yaw/roll escape under load.');(O/'Detachable guide Y restraint.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
