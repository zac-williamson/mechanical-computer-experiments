from pathlib import Path
import os
import numpy as np,trimesh,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R0=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R0/'neck-candidate')
s=(R0/'validate.py').read_text().split('printed_hits=[]')[0];exec(compile(s,'load','exec'));by={p['name']:p for p in parts}
fig,axs=plt.subplots(1,4,figsize=(17,5))
for ax,axis,pos,uv in zip(axs,[0,1,1,1],[0,3,7,10.2],[(1,2),(0,2),(0,2),(0,2)]):
 for n,c in [('Carriage body','teal'),('Carriage bearing end','blue'),('L099','black'),('L097','gray'),('U015','gold')]:
  t=by[n]['mesh'];nn=np.eye(3)[axis];ss=trimesh.intersections.mesh_plane(t,nn,nn*pos)
  for line in ss:ax.plot(line[:,uv[0]],line[:,uv[1]],color=c,lw=1)
 ax.set_aspect('equal');ax.grid();ax.set_xlabel('XYZ'[uv[0]]);ax.set_ylabel('XYZ'[uv[1]]);ax.set_title('XYZ'[axis]+'='+str(pos));ax.set_ylim(-10,23);ax.set_xlim((-1,23) if axis==0 else (-18,18))
fig.tight_layout();fig.savefig(R0/'fork-brace/Sections.png',dpi=130)
# radial section values for full-rotation clutch profile
s=by['L099']['s']
for x in [-8,-6,-4,-2,-1.6,0,1.6,2,4,6,8]:
 # Extract the actual mesh envelope by radial maximum of section lines.
 t=by['L099']['mesh'];sec=trimesh.intersections.mesh_plane(t,[1,0,0],[x,0,0]);print('CLUTCH',x, np.linalg.norm(sec.reshape(-1,3)[:,1:]-[10.2,0],axis=1).max() if len(sec) else None,flush=True)
