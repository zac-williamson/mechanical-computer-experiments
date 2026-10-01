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
 s-=cy(2.5,-8.1,8.1,y,z)+cy(3.3,-.4,.4,y,z)
rows=[]
for name,part,rot in [('left',s^box([-100,-100,-100],[-.1,100,100]),[0,90,0]),('right',s^box([.1,-100,-100],[100,100,100]),[0,-90,0])]:
 mm=part.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts);t.export(O/(name+'.stl'))
 oriented=part.rotate(rot);bb=oriented.bounding_box();oriented=oriented.translate([-bb[0],-bb[1],-bb[2]])
 layers=[]
 for h in np.arange(.3,oriented.bounding_box()[5],.2):
  prior=oriented.slice(h-.2);current=oriented.slice(h);extra=current-prior.offset(.21)
  if extra.area()>.5:layers.append(dict(height=round(float(h),2),unsupported_area=extra.area(),islands=[c.area() for c in current.decompose() if (c^prior.offset(.21)).area()<.01 and c.area()>.1]))
 rows.append(dict(part=name,solids=[c.volume() for c in part.decompose()],layers=layers))
(O/'Split screening.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2),flush=True)
