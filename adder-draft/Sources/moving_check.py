import os
from pathlib import Path
import json,sys,math,itertools,re
import numpy as np
import manifold3d as m
import trimesh
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT;D=json.loads((OUT/'Assembly manifest.json').read_text())
ss={};meta={p['id']:p for p in D['prints']}
for name in meta:
 t=trimesh.load(OUT/(name+'.stl'));ss[name]=m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
h=(ROOT/'Sources/Inputs/multiplexer/Viewer.html').read_text();trace=json.loads(re.search(r'const POSES=(\[.*?\]);',h,re.S)[1]);print('trace',len(trace),str(trace[0])[:200],flush=True)
# Fixed-frame and moving-pair interference in valid states. Moving bolt position is constrained by W.
from motion import transform
def posed(name,state):
 p=meta[name];st={k:dict(q=v[0],b=v[1],w=0,g=0,offset=0) for k,v in state.items()};T=transform(p['actor'],p['motion'],st,D['actors']);return ss[name].transform(T[:3])
# q / lever pairs from final multiplexer contact trace.
pts=[]
for p in trace:
 if isinstance(p,dict):pts.append((p['q'],p.get('b',p.get('beta',0))))
 else:pts.append((p[0],p[1]))
print('q/b',min(x[0] for x in pts),max(x[0] for x in pts),min(x[1] for x in pts),max(x[1] for x in pts),flush=True)
ends={0:min(pts,key=lambda x:abs(x[0]-4.325)),1:min(pts,key=lambda x:abs(x[0]+4.35))}
poses=[]
for ac in D['actors']:
 for p in pts[::15]:
  st={k:ends[0] for k in D['actors']};st[ac]=p;poses.append(st)
for bs in itertools.product([0,1],repeat=4):poses.append({ac:ends[b] for ac,b in zip(D['actors'],bs)})
# Fixed pairs once, dynamic pairs for each pose; touching mating faces are allowed.
fixed=[n for n,p in meta.items() if p['motion']=='fixed'];moving=[n for n in meta if n not in fixed]
hits={}
def bounds_overlap(a,b):
 aa=np.asarray(a.bounding_box());bb=np.asarray(b.bounding_box());return bool(np.all(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])>1e-5))
def test(a,x,b,y,i,state):
 if not bounds_overlap(x,y):return
 v=(x^y).volume()
 if v>1e-3:
  key=a+' / '+b
  if key not in hits or v>hits[key]['volume']:hits[key]=dict(volume=v,pose=i,state=state)
for a,b in itertools.combinations(fixed,2):test(a,ss[a],b,ss[b],-1,{})
for i,state in enumerate(poses):
 cur={n:posed(n,state) for n in moving}
 for a,x in cur.items():
  for b in fixed:test(a,x,b,ss[b],i,state)
 for a,b in itertools.combinations(moving,2):test(a,cur[a],b,cur[b],i,state)
 if i%100==0:print('pose',i,'hits',len(hits),flush=True)
(OUT/'Printed clearance checks.json').write_text(json.dumps(dict(poses=len(poses),ends=ends,worst=hits),indent=2)+'\n');print(json.dumps(hits,indent=2),flush=True)
