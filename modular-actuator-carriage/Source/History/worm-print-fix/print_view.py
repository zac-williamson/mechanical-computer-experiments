from pathlib import Path
import numpy as np,trimesh,json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
R=Path(__file__).resolve().parents[1];O=R/'worm-candidate';meshes=[];cursor=0
fig=plt.figure(figsize=(13,6),facecolor='#e5ebe7')
for i,(name,sign) in enumerate([('Carriage body',1),('Carriage bearing end',-1)]):
 t=trimesh.load(O/(name+'.stl'));M=trimesh.geometry.align_vectors([sign,0,0],[0,0,-1]);t.apply_transform(M);lo=t.bounds[0].copy();t.apply_translation(-lo)
 centre=(M@np.array([sign*9.1,10.2,16,1]))[:3]-lo
 c=t.triangles_center;radius=np.linalg.norm(c[:,:2]-centre[:2],axis=1);contact=(abs(c[:,2]-7.15)<.002)&(t.face_normals[:,2]>.99)&(radius<5.2)
 ax=fig.add_subplot(1,2,i+1,projection='3d');ax.set_facecolor('#e5ebe7');colors=np.tile([.12,.53,.62,1.],(len(t.faces),1));colors[contact]=[1.,.58,.08,1.]
 ax.add_collection3d(Poly3DCollection(t.triangles,facecolors=colors,edgecolor='none',shade=True))
 w,h,z=t.extents;bed=np.array([[[0,0,0],[w,0,0],[w,h,0],[0,h,0]]]);ax.add_collection3d(Poly3DCollection(bed,facecolor=(.3,.35,.35,.12)))
 ax.set(xlim=(0,w),ylim=(0,h),zlim=(0,max(25,z)),xlabel='Print X',ylabel='Print Y',zlabel='Print Z');ax.set_box_aspect((w,h,max(25,z)));ax.view_init(elev=42,azim=-68);ax.set_title(name+'\nWorm contact face upward (orange)',fontsize=12)
 t.apply_translation([cursor,0,0]);cursor+=w+8;meshes.append(t)
layout=trimesh.util.concatenate(meshes);layout.export(O/'Carriage print layout.stl');assert layout.is_watertight and len(layout.split())==2
(O/'Carriage print layout checks.json').write_text(json.dumps(dict(dimensions_mm=layout.extents.tolist(),solids=2,watertight=True,orientations={'Carriage body':'outer +X face on bed','Carriage bearing end':'outer -X face on bed'}),indent=2))
fig.suptitle('Bearing bores vertical · solid material beneath each worm contact face · no support contact on the bearings',fontsize=12);fig.tight_layout();fig.savefig(O/'Carriage print orientations.png',dpi=130);print('Carriage print layout and inspection image saved',flush=True)
