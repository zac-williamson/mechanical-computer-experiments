from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from wall_print_geometry import solid
O=Path(__file__).resolve().parents[1]/'Functional register';P={p['id']:p for p in json.load(open(O/'parts.json'))};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
for r in json.load(open(O/'Candidate manufacturing checks.json'))['parts']:
 if r['print_geometry_pass']:continue
 p=P[r['part']];v=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(v,np.arange(len(v)).reshape(-1,3),process=True);t.apply_transform(r['assembly_to_print_rotation']);t.apply_translation(r['translation_mm']);s=solid(t)
 for fail in r['layer_growth_failures']:
  z=fail['height_mm'];bad=s.slice(z)-s.slice(z-.2).offset(.202,m.JoinType.Round,2,32)
  xy=np.concatenate(bad.to_polygons());xyz=np.c_[xy,np.full(len(xy),z)]-np.array(r['translation_mm']);xyz=trimesh.transform_points(xyz,np.linalg.inv(r['assembly_to_print_rotation']))
  print(r['part'],z,bad.area(),xyz.min(0),xyz.max(0),flush=True)
