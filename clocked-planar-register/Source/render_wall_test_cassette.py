"""Read-only CPU preview of the real cassette meshes; no browser or slicer."""
from pathlib import Path
import json,numpy as np
from PIL import Image,ImageDraw,ImageFont
O=Path(__file__).resolve().parents[1]/'Wall register/Storage test cassette';P=json.load(open(O/'parts.json'));V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
W,H=1560,760;im=Image.new('RGB',(W,H),'#f4f4ed');d=ImageDraw.Draw(im)
try:font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',19);small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',15)
except OSError:font=small=ImageFont.load_default()
for panel,(name,az,el) in enumerate([('Front: XZ plane',0,0),('End: YZ plane',np.pi/2,0),('Oblique',-.72,.38)]):
 def project(a):
  x=np.cos(az)*a[...,0]+np.sin(az)*a[...,1];y=-np.sin(az)*a[...,0]+np.cos(az)*a[...,1]
  return np.stack([x,np.sin(el)*y+np.cos(el)*a[...,2],np.cos(el)*y-np.sin(el)*a[...,2]],axis=-1)
 vv=project(V);lo=vv.min(0);hi=vv.max(0);scale=min(450/(hi[0]-lo[0]),530/(hi[1]-lo[1]));cx=panel*520+260;cy=320;centre=(hi+lo)/2
 polygons=[]
 for p in P:
  a=V[p['offset']//3:p['offset']//3+p['vertices']].reshape(-1,3,3);q=project(a);norm=np.cross(a[:,1]-a[:,0],a[:,2]-a[:,0]);norm/=np.maximum(np.linalg.norm(norm,axis=1)[:,None],1e-12);light=.6+.4*np.abs(norm@np.array([.4,-.7,.6])/np.linalg.norm([.4,-.7,.6]));colors=np.clip(np.array(p['color'])*light[:,None]*255,0,255).astype(int)
  for tri,col in zip(q,colors):polygons.append((tri[:,2].mean(),[(cx+(x-centre[0])*scale,cy-(z-centre[1])*scale) for x,z,_ in tri],tuple(col)))
 for _,points,col in sorted(polygons,key=lambda x:-x[0]):d.polygon(points,fill=col)
 d.text((panel*520+20,18),name,fill='#243b3e',font=font)
 ox,oy=panel*520+70,654
 for axis,label,color in [(np.array([1,0,0]),'+X','#b62b35'),(np.array([0,1,0]),'+Y','#268044'),(np.array([0,0,1]),'+Z','#2756b0')]:
  x,z,depth=project(axis);tx,ty=ox+35*x,oy-35*z;d.line((ox,oy,tx,ty),fill=color,width=3);l=np.hypot(x,z)
  if l<.12:d.ellipse((ox-3,oy-3,ox+3,oy+3),outline=color,width=2);tx+=8;ty+=10
  else:tx+=x*9/max(l,.01);ty-=z*9/max(l,.01)
  d.text((tx-6,ty-8),label,fill=color,font=small)
 d.line((panel*520+519,0,panel*520+519,705),fill='#c3c9c4')
d.text((20,716),'X: data/power axles    Z: mounting rows    +Y: rear/base    |    Actual production meshes; hand-operated test fixture',fill='#243b3e',font=small)
im.save(O/'Preview.png');print(O/'Preview.png')
