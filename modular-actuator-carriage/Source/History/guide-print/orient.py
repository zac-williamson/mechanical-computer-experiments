from pathlib import Path
import trimesh,numpy as np
from PIL import Image,ImageDraw
R=Path(__file__).resolve().parents[1];t=trimesh.load_mesh(R/'neck-candidate/Locking bolt guide.stl')
for normal in ([0,0,-1],[0,1,0],[0,-1,0]):
 bad=(t.face_normals@normal>.7072);bed=(t.triangles@normal).min(axis=1)>(t.vertices@normal).max()-.001;bad&=~bed
 print(normal,'bad area',t.area_faces[bad].sum(),flush=True)
 if normal==[0,1,0]:
  for ids in t.facets:
   if bad[ids[0]] and t.area_faces[ids].sum()>.2:print('FACE',t.face_normals[ids[0]].round(3),round(t.area_faces[ids].sum(),3),t.triangles[ids].min((0,1)).round(3),t.triangles[ids].max((0,1)).round(3),flush=True)
tri=t.triangles;img=Image.new('RGB',(1200,700),'#e4eae6');d=ImageDraw.Draw(img)
for k,M in enumerate([np.array([[1,0,0],[0,0,1],[0,-1,0]]),np.array([[.8,.6,0],[-.3,.4,.866],[.52,-.693,.5]])]):
 a=tri@M.T;b=np.array([a.min((0,1)),a.max((0,1))]);xy=(a[:,:,:2]-b.mean(0)[:2])*[12,-12]+[300+600*k,340]
 for i in np.argsort(a[:,:,2].mean(1)):
  shade=.6+.4*abs(t.face_normals[i]@np.array([.4,.3,.866]));c=np.array([25,119,135])*shade
  d.polygon([tuple(q) for q in xy[i]],fill=tuple(c.astype(int)))
 d.text((20+k*600,20),'X horizontal / Z vertical; +Y rear' if k==0 else 'Oblique',fill='black')
img.save(R/'guide-print/before.png')
