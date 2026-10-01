from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
from PIL import Image,ImageDraw
R=Path(__file__).resolve().parents[1];A=R/'simplified-carriage-r1';out=R/'cheek-strength'
def load(n):
 t=trimesh.load(A/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
im=Image.new('RGB',(1200,700),'white');d=ImageDraw.Draw(im)
for col,(name,heights) in enumerate([('Upper actuator cheek',[20.5,22,24]),('Lower actuator cheek',[7.7,9,11])]):
 s=load(name);sec=s.slice(heights[0]);print(name,'slice bounds',sec.bounds(),flush=True)
 for poly in sec.to_polygons():
  pts=[(col*600+90+x*10,590-y*10) for x,y in poly];d.polygon(pts,fill='#167684',outline='black')
 for x in range(-5,41,5):d.text((col*600+90+x*10,600),str(x),fill='black')
 for y in range(10,46,5):d.text((col*600+10,590-y*10),str(y),fill='black')
 d.text((col*600+30,30),name+' section: horizontal X / vertical Y',fill='black')
 print(name,'erosion components',[(r,len(sec.offset(-r).decompose())) for r in [.5,1,1.5,2]],flush=True)
im.save(out/'Cheek sections.png')
