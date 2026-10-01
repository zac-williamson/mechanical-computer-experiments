from pathlib import Path
import numpy as np,trimesh,manifold3d as m,math
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[1];O=R/'fork-brace-candidate'
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
old=trimesh.load_mesh(O/'baseline/Carriage body.stl');new=trimesh.load_mesh(O/'Carriage body.stl');msh=(solid(new)-solid(old)).to_mesh64();extra=trimesh.Trimesh(msh.vert_properties[:,:3],msh.tri_verts,process=True)
img=Image.new('RGB',(1400,780),'#e4eae6');d=ImageDraw.Draw(img);font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',24);small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',17)
az=-.72;el=.25
M=np.array([[math.cos(az),math.sin(az),0],[-math.sin(el)*math.sin(az),math.sin(el)*math.cos(az),math.cos(el)],[-math.cos(el)*math.sin(az),math.cos(el)*math.cos(az),-math.sin(el)]])
for panel,(label,meshes,cs) in enumerate([('Previous fork connection',[old],[[.10,.48,.55]]),('Braced fork connection',[old,extra],[[.10,.48,.55],[.98,.60,.12]])]):
 tri=np.concatenate([t.triangles for t in meshes]);cols=np.concatenate([np.tile(c,(len(t.faces),1)) for t,c in zip(meshes,cs)]);a=tri@M.T;bb=np.array([(new.vertices@M.T).min(0),(new.vertices@M.T).max(0)]);centre=bb.mean(0);scale=min(570/(bb[1,0]-bb[0,0]),610/(bb[1,1]-bb[0,1]));xy=(a[:,:,:2]-centre[:2])*[scale,-scale]+[panel*700+350,385];norm=np.cross(a[:,1]-a[:,0],a[:,2]-a[:,0]);norm/=np.maximum(np.linalg.norm(norm,axis=1,keepdims=True),1e-12);shade=.55+.45*np.abs(norm@np.array([.3,.4,.866]));colors=np.clip(cols*shade[:,None]*255,0,255).astype(int)
 for i in np.argsort(-a[:,:,2].mean(1)):d.polygon([tuple(q) for q in xy[i]],fill=tuple(colors[i]))
 d.text((panel*700+25,20),label,font=font,fill='#234247');origin=np.array([panel*700+600,680]);d.rounded_rectangle((panel*700+520,620,panel*700+690,740),8,fill='#f9fbf4')
 for axis,c in enumerate(['#b32d31','#288044','#245dc0']):
  end=origin+M[:2,axis]*[40,-40];d.line([tuple(origin),tuple(end)],fill=c,width=3);d.text(tuple(end+[4,2]),['+X','+Y','+Z'][axis],font=small,fill=c)
d.text((25,752),'Gold: added ribs. X follows rods; Z indexes rows; +Y points toward rear/base.',font=small,fill='#234247');img.save(O/'Fork brace comparison.png')
