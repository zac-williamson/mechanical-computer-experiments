import os
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
ROOT=Path(__file__).resolve().parents[1];O=ROOT/'outputs/Adder redesign';D=json.loads((O/'Assembly manifest.json').read_text())
T={p['id']:trimesh.load(O/p['path']) for p in D['prints'] if p['motion']=='fixed' and p['id']!='Base'}
def s(t):return m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
sol={n:s(t) for n,t in T.items()};hits=[]
for i,(n,a) in enumerate(T.items()):
 for k,b in list(T.items())[i+1:]:
  if np.any(np.minimum(a.bounds[1],b.bounds[1])-np.maximum(a.bounds[0],b.bounds[0])<=.001):continue
  v=(sol[n]^sol[k]).volume()
  if v>.01:hits.append([n,k,round(v,3)])
print(hits);(O/'Fixed part checks.json').write_text(json.dumps(hits,indent=2))
