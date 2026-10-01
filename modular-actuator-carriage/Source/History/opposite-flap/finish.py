from pathlib import Path
import json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'opposite-flap-candidate'
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cx(r,a,b,y,z):return m.Manifold.cylinder(b-a,r,circular_segments=64).rotate([0,90,0]).translate([a,y,z])
reports=[];plate=[];cursor=0
for name,sign in [('Carriage body',1),('Carriage bearing end',-1)]:
 t=trimesh.load_mesh(O/(name+'.stl'));new=solid(t);old=solid(trimesh.load_mesh(O/'baseline'/(name+'.stl')));delta=(old-new)+(new-old)
 xa,xb=sorted([sign*9.101,sign*16.249]);regions={'worm bearing':cx(5.1,xa,xb,10.2,16),'clutch contact':cx(6.8,-1.599,1.599,10.2,0),'locking detents':box([-12,23,35.7],[3,30,39.5]),'pin mount':box([-17,16.5,27],[17,22.4,37])}
 if sign==1:
  profile=m.CrossSection([[[-1.6,7.55],[6.95,7.55],[10.7,11.3],[12,11.3],[12,13.05],[10.7,13.05],[6.95,9.3],[-1.6,9.3]]])
  regions['fork braces']=sum((profile.extrude(b-a).rotate([90,0,0]).translate([0,b,0]) for a,b in [(1.55,3.75),(10.5,12.7)]),m.Manifold())
  regions['fork root']=box([-1.6,1.55,7.35],[1.6,12.7,9.3])
 checks={k:(delta^q).volume() for k,q in regions.items()};assert all(v<.001 for v in checks.values()),checks
 assert (new-old).volume()<.001
 p=trimesh.load_mesh(O/(name+' print.stl'));s=solid(p);islands=[]
 for h in np.arange(.16,p.extents[2],.16):
  prev=s.slice(h-.16)
  for c in s.slice(h).decompose():
   if c.area()>.1 and (c^prev.offset(.161,m.JoinType.Round,circular_segments=64)).area()<.01:islands.append(float(h))
 assert not islands,islands
 reports.append(dict(part=name,preserved_regions_mm3=checks,unsupported_island_layers=islands,watertight=t.is_watertight,solids=len(t.split()),added_mm3=(new-old).volume()))
 p.apply_translation(-p.bounds[0]);p.apply_translation([cursor,0,0]);cursor+=p.extents[0]+8;plate.append(p)
trimesh.util.concatenate(plate).export(O/'Carriage print layout.stl',file_type='stl_ascii')
(O/'Opposite flap checks.json').write_text(json.dumps(dict(parts=reports,clearance_basis='Strict material removal only, with zero added volume and unchanged working interfaces. Existing motion and assembly clearance therefore remain valid.',limitations='Print and physical strength remain untested.'),indent=2));print(reports,flush=True)
runpy.run_path(str(R/'opposite-flap/render.py'),run_name='__main__')
