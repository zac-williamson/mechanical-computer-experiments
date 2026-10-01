from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
r=Path(__file__).resolve().parents[1]/'cam-tip-candidate'
def load(p):
 t=trimesh.load_mesh(p);return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
a=load(r/'baseline/Lock control rod.stl');b=load(r/'Lock control rod.stl');d=a-b
region=m.Manifold.cube([16,20,20]).translate([-8,1,39])
report=dict(added_mm3=(b-a).volume(),central_cam_change_mm3=(d^region).volume(),removed_mm3=d.volume(),basis='Removal only: cannot introduce interference. Central working cam X=-8..8 unchanged.')
assert report['added_mm3']<.001 and report['central_cam_change_mm3']<.001,report
(r/'Cam tip checks.json').write_text(json.dumps(report,indent=2));print(report)
