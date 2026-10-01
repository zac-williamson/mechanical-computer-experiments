from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1];O=R/'detent-candidate'
def solid(n):
 t=trimesh.load(O/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
bolt=solid('Locking bolt');end=solid('Carriage bearing end');body=solid('Carriage body');fork=end+body;checks=[]
for endpoint in [-3.75,3.75]:
 for error in [-.19,0,.19]:
  q=endpoint+error;hit=(bolt^fork.translate([q,0,0])).volume();checks.append(dict(carriage_X=q,locked_overlap_mm3=hit));assert hit<.02
 # Beyond play, either travel direction must meet the SAME end piece.
 for dx in [-.3,.3]:
  q=endpoint+dx;v=(bolt^end.translate([q,0,0])).volume();other=(bolt^body.translate([q,0,0])).volume();assert v>.01 and other<.001
  checks.append(dict(carriage_X=q,stop_contact_part='Carriage bearing end',stop_overlap_mm3=v,body_overlap_mm3=other))
for q in np.linspace(-3.75,3.75,31):assert (bolt.translate([0,0,3.8])^fork.translate([float(q),0,0])).volume()<.02
(O/'Detent function checks.json').write_text(json.dumps(dict(checks=checks,release_positions=31,passed=True),indent=2));print('Both stops of both detents belong to end; lock and release checks pass.',flush=True)
