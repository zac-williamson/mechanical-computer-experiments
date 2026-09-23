from pathlib import Path
import json,numpy as np,trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
R=Path(__file__).resolve().parents[1];O=R/'Planar register';B=R.parent/'work/register-mux-reference/multiplexer';rows=[]
for bank in ['Memory','Write']:
 for part in ['Carriage fork and roof','Right carriage bearing support']:
  source=trimesh.load(B/(part+'.stl'));t=trimesh.load(O/(bank+' — '+part+'.stl'));bed=float(source.bounds[0,0]+(96.8 if bank=='Write' else 0));rows.append(dict(part=bank+' — '+part,source_bearing_bed_plane_X_mm=bed,current_min_X_mm=float(t.bounds[0,0]),bearing_face_lift_from_bed_mm=bed-float(t.bounds[0,0])))
t=trimesh.load(O/'Prototype print parts/Write — Direct cam plate.stl');tri=t.triangles;area=float(t.area_faces[np.all(abs(tri[:,:,2])<1e-5,axis=1)].sum());foot=unary_union([Polygon(a[:,:2]) for a in tri if Polygon(a[:,:2]).area>1e-10]).area
report=dict(bearing_faces=rows,cam_plate_print_thickness_mm=float(t.extents[2]),flat_bed_contact_fraction=area/foot,cam_plate_bed_face='Y31 rear face',complete_write_carriage_max_Y_mm=31,attachment='Two LEGO2780 friction pins plus integral shear key; separate plate preserves both original bearing bed faces.',limitations='Geometric bed and mesh checks; pin fits, sliced toolpaths and physical printing not qualified.')
assert all(abs(r['bearing_face_lift_from_bed_mm'])<1e-5 for r in rows)
assert abs(t.extents[2]-5.4)<1e-5
assert area/foot>.999
(O/'Carriage printing checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
