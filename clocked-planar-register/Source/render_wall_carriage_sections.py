"""Render actual right-half guide sections, using exported assembly geometry."""
from pathlib import Path
import json
import numpy as np,trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).resolve().parents[1]/'Wall register';P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
fig,axes=plt.subplots(2,2,figsize=(11,9));fig.patch.set_facecolor('#f6f3eb')
for ax,(bank,x,z,flip,rot,shift) in zip(axes.flat,[('master',0,0,False,np.eye(3),np.zeros(3)),('slave',106,0,False,np.eye(3),np.zeros(3)),('clock',40,-16,True,np.array([[0,0,1],[0,1,0],[-1,0,0]]),np.array([-100,-4,-64])),('write',-76,-16,True,np.array([[0,0,1],[0,1,0],[-1,0,0]]),np.array([-104,-12,-260]))]):
 ax.set_facecolor('#f6f3eb');sign=np.array([-1,1,-1]) if flip else np.ones(3)
 for p in P:
  if p['kind']!='printed' or p['module']!=('bit' if bank in ['master','slave'] else 'control'):continue
  moving=p.get('baseline_id')==bank+' Right carriage bearing support'
  if not moving and p.get('motion','fixed')!='fixed':continue
  a=V[p['offset']//3:p['offset']//3+p['vertices']];a=((a-shift)@rot-np.array([x,0,z]))*sign
  if not a[:,0].min()<12<a[:,0].max():continue
  t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);path=t.section(plane_normal=[1,0,0],plane_origin=[12,0,0])
  if path:
   for curve in path.discrete:ax.plot(curve[:,1],curve[:,2],color='#bc722f' if moving else '#477e83',lw=1.5)
 ax.set(xlim=(10,39),ylim=(3,19),aspect='equal',title=bank.upper(),xlabel='Depth (mm)',ylabel='Height (mm)');ax.grid(alpha=.15)
fig.suptitle('Actual carriage / backing sections\nOrange: moving carriage · Teal: fixed guide and backing',fontsize=14)
fig.tight_layout(rect=(0,0,1,.93));fig.savefig(O/'Carriage sections.png',dpi=160);plt.close(fig)
