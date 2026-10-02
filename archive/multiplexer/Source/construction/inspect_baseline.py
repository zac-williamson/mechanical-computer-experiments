from pathlib import Path
import json,re,gzip,base64,numpy as np,trimesh
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
ROOT=Path(__file__).resolve().parents[1]; OUT=Path(__file__).resolve().parent
D=json.loads(re.search('id="data">(.*?)</script>',(ROOT/'Viewer.html').read_text()).group(1)); RAW=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4')
def meshes():
 return {p['id']:(p,RAW[p['offset']:p['offset']+p['vertices']*3].reshape(-1,3).copy()) for p in D['parts']}
def plot(items,path):
 fig,axs=plt.subplots(1,3,figsize=(18,7))
 for ax,(i,j,k,label) in zip(axs,[(0,2,1,'XZ — viewed from negative Y'),(0,1,2,'XY'),(2,1,0,'ZY')]):
  allv=[]
  for p,v in sorted(items,key=lambda a: -a[1][:,k].mean()):
   tri=v.reshape(-1,3,3);order=np.argsort(-tri[:,:,k].mean(1));tri=tri[order]
   co=np.array(p['color'])/255
   ax.add_collection(PolyCollection(tri[:,:,[i,j]],facecolors=[co],edgecolors='none',alpha=1));allv.append(v)
  vv=np.concatenate(allv);ax.set_xlim(vv[:,i].min()-5,vv[:,i].max()+5);ax.set_ylim(vv[:,j].min()-5,vv[:,j].max()+5);ax.set_aspect('equal');ax.set_title(label);ax.set_xlabel('XYZ'[i]+' mm');ax.set_ylabel('XYZ'[j]+' mm');ax.grid(alpha=.2)
  if j==1:ax.invert_yaxis()
 fig.tight_layout();fig.savefig(path,dpi=130);plt.close(fig)
if __name__=='__main__':
 M=meshes();plot(list(M.values()),OUT/'baseline-orthographic.png')
 for name in ['Left carriage half','Rear bridge and band anchor','Base','Front actuator bridge']:
  plot([M[name]],OUT/(name+'.png'))
