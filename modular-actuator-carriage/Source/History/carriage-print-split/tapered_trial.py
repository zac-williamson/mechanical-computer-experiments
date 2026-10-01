from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
R=Path(__file__).resolve().parents[1];O=Path(__file__).resolve().parent

def load(n):
 t=trimesh.load(R/'adapted'/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,y,z):return m.Manifold.cylinder(b-a,r,circular_segments=48).rotate([0,90,0]).translate([a,y,z])
s=load('Carriage body')+load('Carriage bearing end')
s+=box([7.7,19.4,-4.8],[8.1,30,7.2])+box([7.7,29.4,26],[8.1,41.4,38])
for y,z in [(24.7,.5),(35.4,32)]:
 s+=cy(2.51,.1,16.25,y,z)+cy(3.31,7.8,8.6,y,z)
 s-=cy(2.5,-9.8,6.4,y,z)+cy(3.3,-2.1,-1.3,y,z)
rows=[]
# Continuous front support and a thick worm tunnel grow from the split faces.
addition=cy(7.5,-16.25,16.25,10.2,16)
addition+=box([-16.25,1.55,10.5],[16.25,4.7,27.7])+box([-16.25,4.6,24.8],[16.25,14,27.7])
addition+=box([7.7,39.3,6],[16.25,43.8,27])
addition-=cy(9.15,-17,17,10.2,0)
clear=sum(((load('Upper actuator cheek')+load('Lower actuator cheek')).translate([float(q),0,0]) for q in np.linspace(-3.85,3.85,9)),m.Manifold())
def axis_z(r,a,b,x,y):return m.Manifold.cylinder(b-a,r,circular_segments=48).translate([x,y,a])
for x,y in [(0,18.2),(13.192323604,26.328448698)]:
 for rad,za,zb in [(3.8,24.3,28.9),(2.7,28.6,30.1),(3.6,5.6,7.6)]:
  clear+=axis_z(rad,za,zb,x-3.85,y)+axis_z(rad,za,zb,x+3.85,y)+box([x-3.85,y-rad,za],[x+3.85,y+rad,zb])
s+=addition-clear
void=cy(2.85,-17,17,10.2,16)+cy(5.3,-7.9,7.9,10.2,16)
for sign in [-1,1]:void+=m.Manifold.cylinder(2.45,5.3,2.85,circular_segments=48).rotate([0,sign*90,0]).translate([sign*7.9,10.2,16])
s-=void
for name,part,rot in [('left',s^box([-100,-100,-100],[-1.8,100,100]),[0,90,0]),('right',s^box([-1.6,-100,-100],[100,100,100]),[0,-90,0])]:
 mm=part.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts);t.export(O/(name+'.stl'))
 oriented=part.rotate(rot);bb=oriented.bounding_box();oriented=oriented.translate([-bb[0],-bb[1],-bb[2]])
 layers=[]
 for h in np.arange(.3,oriented.bounding_box()[5],.2):
  prior=oriented.slice(h-.2);current=oriented.slice(h);extra=current-prior.offset(.21)
  if extra.area()>.5:
   pts=np.concatenate(extra.to_polygons());bounds=part.bounding_box()
   layers.append(dict(height=round(float(h),2),model_x=(bounds[3]-float(h) if name=='left' else bounds[0]+float(h)),yz_bounds=[bounds[1]+float(pts[:,1].min()),(bounds[2]+float(pts[:,0].min()) if name=='left' else bounds[5]-float(pts[:,0].max())),bounds[1]+float(pts[:,1].max()),(bounds[2]+float(pts[:,0].max()) if name=='left' else bounds[5]-float(pts[:,0].min()))],unsupported_area=extra.area(),islands=[c.area() for c in current.decompose() if (c^prior.offset(.21)).area()<.01 and c.area()>.1]))
 rows.append(dict(part=name,solids=[c.volume() for c in part.decompose()],layers=layers))
(O/'Split screening.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2),flush=True)
