from pathlib import Path
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parent
for n in ['Carriage body','Carriage bearing end']:
 t=trimesh.load(R/(n+' before.stl'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 s-=m.Manifold.cube([34,15.4,15]).translate([-17,-1,25])
 print(n,[(a.volume(),a.bounding_box()) for a in s.decompose()],flush=True)
 a,b=(-16.25,-9.1) if n=='Carriage body' else (9.1,16.25)
 web=m.Manifold.cube([b-a,5.9,9.7]).translate([a,13.5,18])
 new=s+web
 print('With bearing-to-boss web',n,[(x.volume(),x.bounding_box()) for x in new.decompose()],flush=True)
 for part in ['Upper actuator cheek','Lower actuator cheek','Actuator lever']:
  t=trimesh.load(R.parent/'adapted'/(part+'.stl'));fixed=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
  print('Web clearance',n,part,[(q,(web.translate([q,0,0])^fixed).volume()) for q in [-3.75,0,3.75]],flush=True)
