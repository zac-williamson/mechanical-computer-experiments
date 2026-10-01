from pathlib import Path
import json, math
import numpy as np
import trimesh
import manifold3d as m
R=Path(__file__).resolve().parents[1]/'neck-candidate'
def load(name):
 t=trimesh.load_mesh(R/(name+'.stl'))
 return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
rod=load('Carriage control rod')
rows=[]
for x,name in [(-11,'Carriage bearing end'),(11,'Carriage body')]:
 part=load(name)
 for sign in [-1,1]:
  def overlap(deg):
   M=trimesh.transformations.rotation_matrix(math.radians(sign*deg),[0,1,0],[x,14.6,32])
   return (part.transform(M[:3])^rod).volume()
  lo,hi=0.,3.
  assert overlap(hi)>.001
  for _ in range(11):
   mid=(lo+hi)/2
   if overlap(mid)>.001:hi=mid
   else:lo=mid
  a=math.radians(hi)
  rows.append(dict(part=name,direction=sign,rod_contact_deg=hi,
   worm_bearing_lateral_shift_mm=16*math.sin(a),
   fork_lateral_shift_at_Z5_mm=27*math.sin(a)))
out=dict(method='Rigid rotation about each ideal pin axis until rod intersection exceeds 0.001 mm3. Other parts are not restraints in this test; displacement is a geometric estimate, not an operating simulation.',results=rows)
Path(__file__).with_name('results.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2),flush=True)
