from pathlib import Path
import os,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'neck-candidate';os.environ['PLANAR_OUTPUT']=str(O)
exec(compile((R/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
for n in ['U015','Carriage body','Carriage bearing end']:
 p=next(p for p in parts if p['name']==n);t=p['mesh'];print(n,'bounds',t.bounds.tolist(),'mates',p.get('mates'),flush=True)
 if n=='U015':print('worm radius',np.linalg.norm(t.vertices[:,1:]-[10.2,16],axis=1).max(),flush=True)
 else:
  for x in [-16.2,-9.2,-9.0,-1.5,9,9.2,16.2,16.3,20,26]:
   s=p['s']^box([x, -100,-100],[x+.01,100,100]);print('slice',x,'bounds',s.bounding_box(),'vol',s.volume(),flush=True)
# Render isolated current carriage, +X view and oblique.
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(13,7));ax=fig.add_subplot(111,projection='3d')
for n,c in [('Carriage body','#18788e'),('Carriage bearing end','#429fb3'),('U015','#b29432')]:
 t=next(p['mesh'] for p in parts if p['name']==n);ax.add_collection3d(Poly3DCollection(t.triangles,facecolor=c,edgecolor='none',alpha=.85))
ax.set(xlim=(-20,30),ylim=(0,48),zlim=(-6,42),xlabel='X',ylabel='Y',zlabel='Z');ax.set_box_aspect((50,48,48));ax.view_init(elev=25,azim=30);fig.savefig(R/'worm-print-fix/Current carriage.png',dpi=140)
