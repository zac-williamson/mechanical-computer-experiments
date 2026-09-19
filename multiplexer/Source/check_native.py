from pathlib import Path
import re,json,gzip,base64,numpy as np,trimesh,sys,collections
from trimesh.collision import CollisionManager
O=Path(__file__).resolve().parents[1];pv=np.array(json.loads((O/'Design parameters.json').read_text())['pivot']);h=(O/'Viewer.html').read_text();data=json.loads(re.search('id="data">(.*?)</script>',h).group(1));raw=np.frombuffer(gzip.decompress(base64.b64decode(data['geometry'])),dtype='<f4')
cm=CollisionManager();meta={};geo={}
for p in data['parts']:
 n=p['id']
 if (O/(n+'.stl')).exists():t=trimesh.load(O/(n+'.stl'),process=False)
 else:
  v=raw[p['offset']:p['offset']+p['vertices']*3].reshape(-1,3);t=trimesh.Trimesh(v,np.arange(len(v)).reshape(-1,3),process=False)
 cm.add_object(n,t);meta[n]=p;geo[n]=t
changes=set(meta)
rows=json.loads((O/'Switching trace.json').read_text())['frames'];hits={};allpairs=set();cases=0
for i,r in enumerate(rows):
 if i%5:continue
 q=r['q'];beta=r['b'];w=r['w'];g=r['g']
 for n,p in meta.items():
  motion=p.get('motion','fixed');T=np.eye(4)
  if motion=='carriage':T[0,3]=q
  elif motion=='rocker':T=trimesh.transformations.rotation_matrix(np.radians(beta),[0,0,1],np.r_[pv,0])
  elif motion=='gear':T=trimesh.transformations.rotation_matrix(np.radians(g),[0,0,1],[0,2.2,32])
  elif motion in ['worm','input']:
   T=trimesh.transformations.rotation_matrix(np.radians(w),[1,0,0],[0,10.2,32])
   if motion=='worm':T[0,3]+=q
  elif p.get('spin'):
   T=trimesh.transformations.rotation_matrix(np.radians(w*p['spin']),[1,0,0],[0,p['axisY'],p['axisZ']])
   if motion=='clutch-ring':T[0,3]+=np.sign(q)*max(abs(q)-.4,0)
  cm.set_transform(n,T)
 yes,pairs=cm.in_collision_internal(return_names=True);allpairs.update(pairs);cases+=1
 for pair in pairs:
  if not changes.intersection(pair):continue
  key=' | '.join(sorted(pair));a=hits.setdefault(key,dict(samples=0,first_pose=r));a['samples']+=1
 if cases%100==0:print(cases,len(hits),flush=True)
print(json.dumps(dict(components=len(meta),samples=cases,contact_pairs=hits),indent=2))
