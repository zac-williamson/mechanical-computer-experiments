import os
from pathlib import Path
import json,sys
import numpy as np,trimesh
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent
stage=sys.argv[1] if len(sys.argv)>1 else 'before';src=R/'before' if stage=='before' else Path(os.environ.get('PLANAR_OUTPUT',str(R.parent/'adapted')))
results=[];meshes=[]
for name in ['Carriage body','Carriage bearing end']:
 t=trimesh.load(src/(name+'.stl'));assert t.is_watertight and t.is_winding_consistent
 ids=np.where(t.area_faces>.04)[0];centres=t.triangles_center[ids];normals=t.face_normals[ids];dist=np.full(len(ids),np.inf)
 for start in range(0,len(ids),96):
  end=min(start+96,len(ids));origins=centres[start:end]-normals[start:end]*.0002
  loc,ray,_=t.ray.intersects_location(origins,-normals[start:end],multiple_hits=False)
  dist[start+ray]=np.linalg.norm(loc-centres[start+ray],axis=1)
 all_dist=np.full(len(t.faces),np.inf);all_dist[ids]=dist
 rows=[dict(face=int(i),thickness=float(d),area=float(t.area_faces[i]),point=t.triangles_center[i].tolist(),normal=t.face_normals[i].tolist()) for i,d in zip(ids,dist) if d<3.1]
 report=dict(part=name,faces_sampled=len(ids),samples=rows)
 results.append(report);meshes.append((t,all_dist))
 # Large exposed patches matter more than tiny edge facets.
 print(name,flush=True)
 for d in sorted(rows,key=lambda x:x['area'],reverse=True)[:45]:print(json.dumps(d),flush=True)
(R/(stage+' thickness.json')).write_text(json.dumps(results,indent=2))
font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',17);small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',13)
img=Image.new('RGB',(1500,1040),'#eef0ec');draw=ImageDraw.Draw(img)
for row,(t,th) in enumerate(meshes):
 tri=t.triangles
 base=np.tile([.16,.52,.57],(len(tri),1));base[th<3.1]=[.9,.68,.16];base[th<2.4]=[.9,.29,.16];base[th<1.2]=[.7,.07,.17]
 for col,(label,az,el) in enumerate([('Front',0,0),('Oblique',-.8,.4),('Rear / underside',2.4,-.55)]):
  M=np.array([[np.cos(az),np.sin(az),0],[-np.sin(el)*np.sin(az),np.sin(el)*np.cos(az),np.cos(el)],[-np.cos(el)*np.sin(az),np.cos(el)*np.cos(az),-np.sin(el)]])
  a=tri@M.T;flat=a.reshape(-1,3);bb=np.array([flat.min(0),flat.max(0)]);centre=bb.mean(0);scale=min(410/(bb[1,0]-bb[0,0]),380/(bb[1,1]-bb[0,1]));xy=(a[:,:,:2]-centre[:2])*[scale,-scale]+[col*500+240,row*500+260]
  n=t.face_normals@M.T;shade=.6+.4*abs(n@np.array([.3,.4,.866]));colors=np.clip(base*shade[:,None]*255,0,255).astype(int)
  for i in np.argsort(-a[:,:,2].mean(1)):draw.polygon([tuple(q) for q in xy[i]],fill=tuple(colors[i]))
  draw.text((col*500+12,row*500+12),results[row]['part']+' · '+label,font=font,fill='#234247')
  origin=np.array([col*500+420,row*500+445]);draw.rectangle((col*500+365,row*500+385,col*500+495,row*500+495),fill='#ffffff')
  for axis,color in enumerate(['#b32d31','#288044','#245dc0']):
   end=origin+M[:2,axis]*[38,-38];draw.line([tuple(origin),tuple(end)],fill=color,width=3);draw.text(tuple(end+[3,2]),['+X','+Y','+Z'][axis],font=small,fill=color)
draw.text((15,1010),'Normal-ray thickness samples: dark red <1.2 mm; red <2.4 mm; amber <3.1 mm. Edge/chamfer samples require interpretation.',font=small,fill='#234247')
img.save(R/(stage+' thickness.png'));print('Audit saved',stage,flush=True)
