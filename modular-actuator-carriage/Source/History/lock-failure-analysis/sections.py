from pathlib import Path
import json,gzip,base64
import numpy as np,trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1];D=json.loads((R/'neck-candidate/Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
fig,axs=plt.subplots(1,3,figsize=(15,6))
for ax,axis,pos,uv in zip(axs,[0,1,2],[-5.05,26,46],[(1,2),(0,2),(0,1)]):
 for name,color in [('Locking bolt','orange'),('Locking bolt guide','black'),('Lock control rod','blue'),('Carriage bearing end','teal')]:
  p=next(p for p in D['parts'] if p['name']==name);a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  n=np.eye(3)[axis];o=n*pos;s=trimesh.intersections.mesh_plane(t,n,o)
  for line in s:ax.plot(line[:,uv[0]],line[:,uv[1]],color=color,lw=2)
  if name=='Locking bolt':print('BOLT SECTION',axis,pos,np.round(s,2).tolist(),flush=True)
 ax.set_aspect('equal');ax.grid();ax.set_title(f'{"XYZ"[axis]} = {pos} mm');ax.set_xlabel('XYZ'[uv[0]]);ax.set_ylabel('XYZ'[uv[1]])
axs[0].set_xlim(0,40);axs[0].set_ylim(34,58);axs[1].set_xlim(-21,21);axs[1].set_ylim(34,58);axs[2].set_xlim(-21,21);axs[2].set_ylim(0,40)
fig.suptitle('Current lock: orange bolt / black guide / blue release rod / teal carriage');fig.tight_layout();fig.savefig(R/'lock-failure-analysis/Sections.png',dpi=130)
