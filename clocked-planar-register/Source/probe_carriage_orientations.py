from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from wall_print_geometry import qualify,solid
O=Path(__file__).resolve().parents[1]/'Wall register';P={p['id']:p for p in json.load(open(O/'parts.json'))};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
p=P['master Right carriage bearing support'];v=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(v,np.arange(len(v)).reshape(-1,3),process=True)
for sign in [-1,1]:
 _,q=qualify(t,0,sign);print('ORIGINAL',sign,q['print_geometry_pass'],q['unsupported_face_area_mm2'],q['layer_growth_failures'],flush=True)
s=solid(t)
def cyl(r,lo,hi):return m.Manifold.cylinder(hi-lo,r,circular_segments=32).rotate([0,90,0]).translate([lo,24,5.3])
s+=m.Manifold.cube([7.6,7.6,8.7]).translate([8,20.2,0]);s-=cyl(3.3,7.9,9.4)+cyl(2.5,7.9,15.7)
s-=m.Manifold.cylinder(.9,3.3,2.5,circular_segments=32).rotate([0,90,0]).translate([9.4,24,5.3])
q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
for sign in [-1,1]:
 _,q=qualify(t,0,sign);print('REVISED',sign,q['print_geometry_pass'],q['unsupported_face_area_mm2'],q['layer_growth_failures'],flush=True)
