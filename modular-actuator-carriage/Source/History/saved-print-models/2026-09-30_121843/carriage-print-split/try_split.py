from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1];O=Path(__file__).resolve().parent

def load(n):
 t=trimesh.load(R/'carriage-print-split/before'/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,y,z):return m.Manifold.cylinder(b-a,r,circular_segments=48).rotate([0,90,0]).translate([a,y,z])
original=load('Carriage body')+load('Carriage bearing end');s=original
s+=box([7.7,19.4,-4.8],[8.1,30,7.2])+box([7.7,29.4,26],[8.1,41.4,38])
for y,z in [(24.7,.5),(35.4,32)]:
 s+=cy(2.51,.1,16.25,y,z)+cy(3.31,7.8,8.6,y,z)
 s-=cy(2.5,-17,27,y,z)+cy(3.3,-2.1,-1.3,y,z)
rows=[]
# Continuous front support and a thick worm tunnel grow from the split faces.
addition=cy(7.5,-16.25,16.25,10.2,16)
addition+=box([-16.25,1.55,10.5],[16.25,4.7,27.7])+box([-16.25,4.6,24.8],[16.25,14,27.7])
addition+=box([-16.25,40.3,-4.8],[16.25,43.8,39.4])
addition+=box([-16.25,24,2],[16.25,43.8,7.2])
addition+=box([-16.25,14.4,26.75],[16.25,22.4,37.25])
addition+=box([-16.25,22.3,35.5],[16.25,43.8,39.4])
for x in [-8.8,-1.3]:addition-=box([x-2.15,23.5,35.8],[x+2.15,28.5,39.5])
for x in [-11,11]:
 addition-=m.Manifold.cylinder(10,2.5,circular_segments=48).rotate([-90,0,0]).translate([x,14.3,32])
 addition-=m.Manifold.cylinder(.8,3.25,circular_segments=48).rotate([-90,0,0]).translate([x,14.3,32])
for y,z in [(24.7,.5),(35.4,32)]:addition-=cy(2.5,-17,27,y,z)+cy(3.3,-2.1,-1.3,y,z)
addition+=box([-16.25,34.4,7.2],[16.25,43.8,35.5])-box([-17,34.3,11.5],[17,37.7,20.4])
for sign in [-1,1]:
 pts=[(sign*8.4,13.9),(sign*16.25,13.9),(sign*16.25,17.5),(sign*12,17.5)]
 if sign<0:pts.reverse()
 addition+=m.CrossSection([pts]).extrude(4.3).translate([0,0,23.4])
addition-=cy(9.15,-17,17,10.2,0)
clear=sum(((load('Upper actuator cheek')+load('Lower actuator cheek')).translate([float(q),0,0]) for q in np.linspace(-3.85,3.85,9)),m.Manifold())
def axis_z(r,a,b,x,y):return m.Manifold.cylinder(b-a,r,circular_segments=48).translate([x,y,a])
for x,y in [(0,18.2),(13.192323604,26.328448698)]:
 for rad,za,zb in [(3.8,24.3,28.9),(2.7,28.6,30.1),(3.6,5.6,7.6),(2.65,7.5,30.1)]:
  clear+=axis_z(rad,za,zb,x-3.85,y)+axis_z(rad,za,zb,x+3.85,y)+box([x-3.85,y-rad,za],[x+3.85,y+rad,zb])
clear+=axis_z(5.2,11.8,20.2,-3.85,18.2)+axis_z(5.2,11.8,20.2,3.85,18.2)+box([-3.85,13,11.8],[3.85,23.4,20.2])
s=original+((s-original)+addition-clear)
void=cy(2.85,-17,17,10.2,16)+cy(5.3,-9.1,9.1,10.2,16)
for sign in [-1,1]:void+=m.Manifold.cylinder(.6,3.45,2.85,circular_segments=48).rotate([0,sign*90,0]).translate([sign*9.1,10.2,16])
s-=void
# Keep the working cam profile; remove only the unused far-left post fringe.
s-=box([-17,35.2,11.5],[-8.1,37.5,20.4])
s-=box([-17,14.3,37.25],[17,20.49,40])
s-=box([4.11,20.49,37.25],[17,22.29,40])
for y,z in [(24.7,.5),(35.4,32)]:
 s-=cy(2.5,-17,27,y,z)+cy(3.3,-2.1,-1.3,y,z)
 for sign,start in [(-1,-2.1),(1,-1.3)]:s-=m.Manifold.cylinder(.8,3.3,2.5,circular_segments=48).rotate([0,sign*90,0]).translate([start,y,z])
for name,part,rot in [('left',s^box([-100,-100,-100],[-1.8,100,100]),[0,90,0]),('right',s^box([-1.6,-100,-100],[100,100,100]),[0,-90,0])]:
 part=part.simplify(.001)
 import ast,sys
 sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
 tree=ast.parse((R/'adapt.py').read_text());defs=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ['solid','mesh','add']]
 env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[])
 exec(compile(ast.Module(body=defs,type_ignores=[]),'export-cleanup','exec'),env)
 env['add'](name,part);t=env['parts'][0]['mesh'];part=env['parts'][0]['s'];t.export(O/(name+'.stl'))
 oriented=part.rotate(rot);bb=oriented.bounding_box();oriented=oriented.translate([-bb[0],-bb[1],-bb[2]])
 layers=[]
 for h in np.arange(.3,oriented.bounding_box()[5],.2):
  prior=oriented.slice(h-.2);current=oriented.slice(h);extra=current-prior.offset(.21)
  if extra.area()>.5:
   pts=np.concatenate(extra.to_polygons());bounds=part.bounding_box()
   layers.append(dict(height=round(float(h),2),model_x=(bounds[3]-float(h) if name=='left' else bounds[0]+float(h)),yz_bounds=[bounds[1]+float(pts[:,1].min()),(bounds[2]+float(pts[:,0].min()) if name=='left' else bounds[5]-float(pts[:,0].max())),bounds[1]+float(pts[:,1].max()),(bounds[2]+float(pts[:,0].max()) if name=='left' else bounds[5]-float(pts[:,0].min()))],unsupported_area=extra.area(),islands=[c.area() for c in current.decompose() if (c^prior.offset(.21)).area()<.01 and c.area()>.1]))
 rows.append(dict(part=name,solids=[c.volume() for c in part.decompose()],layers=layers))
(O/'Split screening.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2),flush=True)

import runpy
if all(len([v for v in r["solids"] if v>1e-5])==1 for r in rows):
 runpy.run_path(str(O/"check_trial.py"),run_name="__main__")
