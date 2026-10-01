import os
from pathlib import Path
import json,gzip,base64,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[1]/'clean-base-candidate'
s=(R/'Viewer.html').read_text();D=json.loads(s.split('<script type="application/json" id="data">',1)[1].split('</script>',1)[0]);v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
D['parts']=[p for p in D['parts'] if p['name'] in ['Module base']]
idx=0;tri=[];cols=[]
for p in D['parts']:
 b=D['bands'][idx].get(p['name'],p);a=v[b['offset']//3:b['offset']//3+b['vertices']].astype(float)
 if b is p:
  M=np.array(D['transforms'][idx][p['name']]).reshape(4,4).T;a=a@M[:3,:3].T+M[:3,3]
 a=a.reshape(-1,3,3);tri.append(a);cols.extend([p['color']]*len(a))
tri=np.concatenate(tri);cols=np.array(cols)
img=Image.new('RGB',(1500,560),'#e4eae6');draw=ImageDraw.Draw(img)
font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',19);small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',14)
for panel,(label,az,el) in enumerate([('Front',0,0),('End',math.pi/2,0),('Oblique',-.4,.35)]):
 M=np.array([[math.cos(az),math.sin(az),0],[-math.sin(el)*math.sin(az),math.sin(el)*math.cos(az),math.cos(el)],[-math.cos(el)*math.sin(az),math.cos(el)*math.cos(az),-math.sin(el)]])
 a=tri@M.T;bounds=np.array([a.reshape(-1,3).min(0),a.reshape(-1,3).max(0)]);centre=bounds.mean(0);scale=min(430/(bounds[1,0]-bounds[0,0]),390/(bounds[1,1]-bounds[0,1]));xy=(a[:,:,:2]-centre[:2])*[scale,-scale]+[panel*500+250,270]
 n=np.cross(a[:,1]-a[:,0],a[:,2]-a[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-12);shade=.5+.5*np.abs(n@np.array([.3,.4,.866]));colors=np.clip(cols*shade[:,None]*255,0,255).astype(int)
 for i in np.argsort(-a[:,:,2].mean(1)):draw.polygon([tuple(q) for q in xy[i]],fill=tuple(colors[i]))
 draw.text((panel*500+20,18),label,font=font,fill='#234247')
 origin=np.array([panel*500+420,460]);draw.rounded_rectangle((panel*500+350,395,panel*500+490,525),8,fill='#f9fbf4')
 for axis,c in enumerate(['#b32d31','#288044','#245dc0']):
  delta=M[:2,axis]*[43,-43];end=origin+delta;draw.line([tuple(origin),tuple(end)],fill=c,width=3);draw.text(tuple(end+[4,2]),['+X','+Y','+Z'][axis],font=small,fill=c)
 draw.line((panel*500+499,10,panel*500+499,530),fill='#c3cdca')
draw.text((20,536),'New base: solid mounting blocks and rear plate · X: rods/axles; Z: rows; +Y: rear/base',font=small,fill='#234247')
img.save(R/'Rebuilt base views.png');print('Saved standalone mesh review views.',flush=True)
