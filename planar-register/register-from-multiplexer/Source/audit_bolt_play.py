from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from direct_cam_math import cam_lift
R=Path(__file__).resolve().parents[1];P=R/'Planar register';O=R/'Cam mechanism audit'
def solid(n):
 t=trimesh.load(P/(n+'.stl'));return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
guide=solid('Memory — Front bearing cheek');bolt=solid('Direct lock bolt');rows=[]
for q in [-.75,-.45,0,.2,1,3.75]:
 up=cam_lift(q);lower=max(45.7,40.9+up);pivot=np.array([38,19.6,(lower+53.3)/2]);roller=np.array([38,27.6,57.3+up])
 for deg in [-3,3]:
  T=trimesh.transformations.rotation_matrix(np.radians(deg),[0,1,0],pivot);T[0,3]+=.15*np.sign(deg)
  placed=bolt.translate([0,0,up]).transform(T[:3]);vol=max(0.,float((placed^guide).volume()));point=trimesh.transform_points([roller],T)[0]
  rows.append(dict(q=q,lift=up,tilt_deg=deg,lateral_offset_mm=.15*np.sign(deg),guide_intersection_mm3=vol,roller_x_displacement_mm=float(point[0]-38)))
r=dict(method='Exact solid intersections of complete bolt and bearing cheek with tilted bolt. Other components/cam are absent: this measures guide freedom, not an assembled collision-free pose. Roller location follows the rigid bolt.',cases=rows,free_cases_exceeding_previous_point6mm_budget=[x for x in rows if x['guide_intersection_mm3']<.005 and abs(x['roller_x_displacement_mm'])>.6])
assert r['free_cases_exceeding_previous_point6mm_budget']
(O/'Bolt guide play.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
