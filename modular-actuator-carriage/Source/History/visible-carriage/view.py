from pathlib import Path
import os,json,gzip,base64,math,runpy
import numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[1];O=R/'visible-carriage-candidate';B=O/'baseline'
p=O/'Viewer.html';s=p.read_text().replace('Compact carriage — keyed rod connection','Open-view carriage').replace('Raised joining-pin mounts removed · two rod pins and locating keys secure the carriage halves.','Rounded sight opening exposes the mechanism in the XZ plane · keyed rod connection retained.').replace('let az=-.4,el=.35,zoom=1','let az=0,el=0,zoom=1').replace('<a href="Rod%20tie%20print%20layout.stl">Compact carriage replacement STL</a>','<a href="Visible%20carriage%20print%20layout.stl">Open-view carriage STL</a>').replace('<a href="Rod%20tie%20notes.md">Keyed rod revision notes</a>','<a href="Visibility%20notes.md">Visibility revision notes</a>');p.write_text(s)
os.environ['PLANAR_OUTPUT']=str(O);runpy.run_path(str(R/'render_review.py'),run_name='__main__')
font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',22);small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',16)
img=Image.new('RGB',(1440,650),'#e4eae6');dr=ImageDraw.Draw(img);counts=[]
for col,(folder,title) in enumerate([(B,'Before'),(O,'Open-view carriage')]):
 s=(folder/'Viewer.html').read_text();D=json.loads(s.split('<script type="application/json" id="data">',1)[1].split('</script>',1)[0]);v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);tris=[];colors=[];ids=[];pose=4
 for part in D['parts']:
  p=D['bands'][pose].get(part['name'],part);a=v[p['offset']//3:p['offset']//3+p['vertices']].astype(float)
  if p is part:
   M=np.array(D['transforms'][pose][part['name']]).reshape(4,4).T;a=a@M[:3,:3].T+M[:3,3]
  a=a.reshape(-1,3,3);tris.extend(a);colors.extend([part['color']]*len(a));ids.extend([1 if part['name'] in ['U015','U022','Actuator lever'] else 0]*len(a))
 tri=np.array(tris);xy=tri[:,:,[0,2]]*[6,-6]+[col*720+360,460];mask=Image.new('L',(720,650),0);md=ImageDraw.Draw(mask)
 norm=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);norm/=np.maximum(np.linalg.norm(norm,axis=1,keepdims=True),1e-12);shade=.55+.45*np.abs(norm@np.array([.3,.85,.4]));colors=(np.clip(np.array(colors)*shade[:,None],0,1)*255).astype(int)
 for i in np.argsort(-tri[:,:,1].mean(axis=1)):
  polygon=[tuple(q) for q in xy[i]];dr.polygon(polygon,fill=tuple(colors[i]));md.polygon([(x-col*720,y) for x,y in polygon],fill=ids[i])
 counts.append(int(np.count_nonzero(np.array(mask))))
 dr.text((col*720+25,25),title,font=font,fill='#234247')
 dr.rounded_rectangle((col*720+555,495,col*720+700,630),radius=9,fill='#f9fbf4');o=(col*720+590,585);dr.line([o,(o[0]+65,o[1])],fill='#b32d31',width=3);dr.line([o,(o[0],o[1]-60)],fill='#245dc0',width=3);dr.text((o[0]+67,o[1]-10),'+X',font=small,fill='#b32d31');dr.text((o[0]-12,o[1]-85),'+Z',font=small,fill='#245dc0');dr.text((col*720+565,602),'+Y into page',font=small,fill='#288044')
dr.text((25,615),'Opaque parts · identical pose and scale · looking from −Y toward the XZ plane',font=small,fill='#234247');img.save(O/'Visibility comparison.png')
(O/'Visibility measurement.json').write_text(json.dumps(dict(before_visible_target_pixels=counts[0],after_visible_target_pixels=counts[1],targets=['worm','reaction gear','actuator lever'],method='Identical orthographic opaque rendering, frame 4, 6 pixels/mm. Pixel area is a view-specific illustration, not structural validation.'),indent=2));print('VISIBLE TARGET PIXELS',counts,flush=True)
