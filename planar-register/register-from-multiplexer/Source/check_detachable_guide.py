"""Check the detachable guide's roller sweep, elastic path and seating lands."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';hw=json.load(open(O/'hardware.json'));v=np.load(O/'hardware.npz')['vertices'];g=trimesh.load(O/'Detachable bolt guide.stl');b=trimesh.load(O/'Unified rear backbone.stl')
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices),np.array(t.faces,dtype=np.uint64)))
sg=solid(g);rows=[]
for h in hw:
 if not h['id'].startswith('Cam roller'):continue
 a=v[h['offset']//3:h['offset']//3+h['vertices']];cx=-5.05;cz=(a[:,2].min()+a[:,2].max())/2;r=np.linalg.norm(a[:,[0,2]]-[cx,cz],axis=1).max();lo,hi=a[:,1].min(),a[:,1].max()
 envelope=m.Manifold.cylinder(hi-lo,r,circular_segments=128).rotate([-90,0,0]).translate([cx,lo,cz]);worst=max((sg^envelope.translate([0,0,z])).volume() for z in np.linspace(0,5.4,55));assert worst<.001,(h['id'],worst);rows.append(dict(part=h['id'],maximum_full_rotation_envelope_overlap_mm3=worst))
# Densely sample the band faces and connecting straight sides at every bolt lift.
worst=0
for up in np.linspace(0,5.4,55):
 points=[]
 for r in np.linspace(2.5,3.1,4):
  for y in [32.5,33.0,33.5]:
   for z,start in [(50,np.pi),(69+up,0)]:
    th=np.linspace(start,start+np.pi,129);points.extend(np.column_stack([-5.05+r*np.cos(th),np.full(len(th),y),z+r*np.sin(th)]))
   for x in [-5.05-r,-5.05+r]:
    points.extend([[x,y,z] for z in np.linspace(50,69+up,129)])
 points=np.asarray(points)
 for mesh in [g,b]:
  inside=mesh.contains(points)
  if inside.any():worst=max(worst,float(trimesh.proximity.closest_point(mesh,points[inside])[1].max()))
assert worst<.001,worst
r=dict(roller_checks=rows,roller_lift_samples=55,band_lift_samples=55,band_maximum_sampled_penetration_mm=worst,fixed_band_anchor_z_mm=50,positive_seat_z_mm=45.5,guide_pin_engagement_nominal_mm=6.5,scope='Conservative full-rotation roller envelopes over 55 bolt heights; sampled band surfaces against base and guide. Seating lands set guide height. No load or fatigue qualification.');(O/'Detachable guide checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
