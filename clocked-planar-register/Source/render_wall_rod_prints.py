"""Show actual oriented CLOCK rod and pickup meshes on their print beds."""
from pathlib import Path
import json,numpy as np,trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
O=Path(__file__).resolve().parents[1]/'Wall register'
fig=plt.figure(figsize=(12,6),facecolor='#f7f5ee')
for i,(name,label) in enumerate([('Control CLOCK direct rod and pickup','CLOCK rod — long sliding face on bed'),('Control CLOCK detachable pickup','Separate pickup — flat rear face on bed')]):
 t=trimesh.load(O/'Print oriented sliding parts'/(name+'.stl'));ax=fig.add_subplot(1,2,i+1,projection='3d');ax.set_facecolor('#f7f5ee')
 normals=t.face_normals;light=.55+.4*abs(normals@np.array([.3,-.4,.8]));colors=light[:,None]*np.array([.83,.47,.19])[None,:]
 ax.add_collection3d(Poly3DCollection(t.triangles,facecolors=colors,linewidths=.1,edgecolors=(.2,.15,.1,.15)))
 lo,hi=t.bounds;bed=np.array([[[lo[0]-3,lo[1]-3,-.05],[hi[0]+3,lo[1]-3,-.05],[hi[0]+3,hi[1]+3,-.05],[lo[0]-3,hi[1]+3,-.05]]]);outline=np.vstack([bed[0],bed[0][0]]);ax.plot(outline[:,0],outline[:,1],outline[:,2],color='#888b83',lw=1)
 ax.set_xlim(lo[0]-3,hi[0]+3);ax.set_ylim(lo[1]-3,hi[1]+3);ax.set_zlim(0,hi[2]+3);ax.set_box_aspect(hi-lo+6);ax.view_init(elev=25,azim=-65);ax.set_yticks(np.linspace(0,hi[1],3));ax.set_title(label,pad=18,fontsize=12);ax.set_xlabel('mm');ax.set_ylabel('mm');ax.set_zlabel('Height (mm)')
fig.suptitle('Print separately, then join with two LEGO friction pins',fontsize=17)
fig.text(.5,.05,'Actual exported print geometry. The rod running faces and pickup slot require no support interface.',ha='center',fontsize=11)
fig.tight_layout(rect=[0,.1,1,.92]);fig.savefig(O/'CLOCK rod print orientation.png',dpi=150)
