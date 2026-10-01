from pathlib import Path
import json
import numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'fork-brace-candidate'
t=trimesh.load_mesh(O/'Carriage body.stl');old=trimesh.load_mesh(O/'baseline/Carriage body.stl')
# +X is downward toward the bed; growth is -X. Compare exposed faces
# against the original part to identify surfaces introduced by these ribs.
ids=np.flatnonzero(t.face_normals[:,0]>.0001)
pts=t.triangles_center[ids]
_,dist,_=trimesh.proximity.closest_point(old,pts)
fresh=ids[dist>.001]
bad=fresh[t.face_normals[fresh,0]>np.sqrt(.5)+.001]
angles=np.degrees(np.arcsin(np.clip(t.face_normals[fresh,0],0,1)))
report=dict(print_bed_face='+X',print_growth_direction='-X',new_downward_surface_count=len(fresh),maximum_new_overhang_from_vertical_deg=float(angles.max()) if len(angles) else 0,new_surface_area_exceeding_45_degrees_mm2=float(t.area_faces[bad].sum()),scope='New fork connection braces only. Original fork contact lip is unchanged; this does not certify all original overhangs.')
print(report,flush=True);assert len(bad)==0
(O/'Fork brace print slope checks.json').write_text(json.dumps(report,indent=2))
