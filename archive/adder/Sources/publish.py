import os
from pathlib import Path
import sys,json,re,gzip,base64,numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT;D=json.loads((OUT/'Assembly manifest.json').read_text());sys.path.insert(0,str(Path(__file__).resolve().parent));from render_ldraw import LDraw,render
from motion import transform
lib=LDraw(str(Path(os.environ.get('LDRAW_PATH','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'));trace=json.loads((ROOT.parent/'multiplexer/Actuator poses.json').read_text());ends={str(k):min(trace,key=lambda p:abs(p['q']-(4.325 if k==0 else -4.35))) for k in [0,1]}
parts=[];chunks=[];offset=0;colors={'X':[161,119,172],'P':[58,137,166],'S':[205,151,54],'C':[92,151,111],'fixed':[107,126,133]}
def emit(n,v,actor,motion,kind):
 global offset
 a=np.asarray(v,dtype='<f4').reshape(-1);col=colors.get(actor,colors['fixed']) if kind!='fixed' else [114,137,129]
 if kind=='pin':col=[60,65,67]
 if n=='Base':col=[121,137,128]
 parts.append(dict(id=n,module=actor,actor=actor,motion=motion,kind=kind,color=col,offset=offset,vertices=len(a)//3));chunks.append(a);offset+=len(a)
for p in D['prints']:emit(p['id'],trimesh.load(OUT/p['path']).triangles,p['actor'],p['motion'],'fixed' if p['motion']=='fixed' else 'moving')
for r in D['records']:
 t,_=lib.mesh(r['part']);v=t*.4@np.array(r['matrix']).reshape(3,3).T+np.array(r['pos'])*.4;emit(r['record_id'],v,r['actor'],r['motion'],'pin' if r['part']=='2780.dat' else 'lego')
# The retained actuator band geometry is shown at the selected zero-input endpoint.
import math
for ac,off in D['actors'].items():
 if ac=='S':continue
 a=np.array([36.96055698703205,-11.721496948498547]);pivot=np.array([14.50092459,-3.26189219]);b=np.array([21.987468724068282,-6.08176044140031]);ang=math.radians(ends['0']['b']);R=np.array([[math.cos(ang),-math.sin(ang)],[math.sin(ang),math.cos(ang)]]);b=pivot+R@(b-pivot);direction=math.atan2(b[1]-a[1],b[0]-a[0]);polys=[]
 for rad in [2.4,1.8]:polys.append(np.array([c+rad*np.array([math.cos(t),math.sin(t)]) for c,start in [(a,direction+math.pi/2),(b,direction-math.pi/2)] for t in np.linspace(start,start+math.pi,25)]))
 band=[];N=len(polys[0]);z=45.5
 for i in range(N):
  j=(i+1)%N
  for zz in [-.6,.6]:band.extend([[np.r_[polys[0][i],z+zz],np.r_[polys[0][j],z+zz],np.r_[polys[1][i],z+zz]],[np.r_[polys[1][i],z+zz],np.r_[polys[0][j],z+zz],np.r_[polys[1][j],z+zz]]])
  for poly in polys:band.extend([[np.r_[poly[i],z-.6],np.r_[poly[j],z-.6],np.r_[poly[i],z+.6]],[np.r_[poly[i],z+.6],np.r_[poly[j],z-.6],np.r_[poly[j],z+.6]]])
 sg=D['orientations'][ac];emit(ac+' band at zero-input pose',np.array(band)@np.diag([sg,1,sg])+off,ac,'fixed','band')
data=dict(parts=parts,geometry=base64.b64encode(gzip.compress(np.concatenate(chunks).tobytes())).decode(),actors=D['actors'],orientations=D['orientations'],ends=ends,ports=D['ports'])
(OUT/'data.js').write_text('const D='+json.dumps(data,separators=(',',':'))+';')
(OUT/'LEGO parts.csv').write_text('Part,Quantity\n'+''.join(f'{p},{n}\n' for p,n in sorted(__import__('collections').Counter(r['part'] for r in D['records']).items())))
print('Viewer geometry generated',len(parts),flush=True)
# Static assembly illustration uses the same settled state as the viewer.
ts=[];cs=[];st={ac:ends['0'] for ac in D['actors']}
for p,a in zip(parts,chunks):
 t=a.reshape(-1,3,3);T=transform(p['actor'],p['motion'],st,D['actors']);ts.append(t@T[:3,:3].T+T[:3,3]);r,g,b=p['color'];cs.append(np.full(len(t),0x02000000|(r<<16)|(g<<8)|b))
class Scene:
 source=OUT/'Assembly manifest.json';root='Compact one-level adder';missing={};warnings=[]
 def mesh(self,root=None):return np.concatenate(ts)*2.5,np.concatenate(cs)
 def rgb(self,c):return[(int(c)>>16)&255,(int(c)>>8)&255,int(c)&255]
render(Scene(),OUT/'Assembly.png',azimuth=130,elevation=48,width=1600,height=1100,supersample=1)
