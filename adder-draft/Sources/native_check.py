import os
from pathlib import Path
import sys,json,re,math,itertools
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT;D=json.loads((OUT/'Assembly manifest.json').read_text());sys.path.insert(0,str(Path(__file__).resolve().parent));from render_ldraw import LDraw
lib=LDraw(str(Path(os.environ.get('LDRAW_PATH','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'));trace=json.loads(re.search(r'const POSES=(\[.*?\]);',(ROOT/'Sources/Inputs/multiplexer/Viewer.html').read_text(),re.S)[1]);ends={0:min(trace,key=lambda p:abs(p['q']-4.325)),1:min(trace,key=lambda p:abs(p['q']+4.35))}
def lift(q):return -5.5*(1-np.clip((q-.5)/3.825,0,1))
def rot(axis,deg):
 a=np.eye(4);a[:3,:3]=trimesh.transformations.rotation_matrix(math.radians(deg),axis)[:3,:3];return a
print('Loading geometry',flush=True)
meshcache={p['id']:trimesh.load(OUT/p['path']) for p in D['prints']};nativecache={}
for ri,r in enumerate(D['records']):
 if ri%40==0:print('Native mesh',ri,flush=True)
 t,_=lib.mesh(r['part']);a=t*.4@np.array(r['matrix']).reshape(3,3).T+np.array(r['pos'])*.4
 nativecache[r['record_id']]=np.unique(np.concatenate([a.reshape(-1,3),a.mean(1)]),axis=0)
from motion import transform
def xform(actor,motion,state,rid=''):return transform(actor,motion,state,D['actors'])
states=[dict(zip(D['actors'],v)) for v in itertools.product([ends[0],ends[1]],repeat=4)]
for ac in D['actors']:
 for q in [-3,-1,0,1,3]:
  st={k:ends[0] for k in D['actors']};st[ac]=min(trace,key=lambda p:abs(p['q']-q));states.append(st)
hits={};tested=0;cache=set()
print('Geometry loaded',flush=True)
for si,st in enumerate(states):
 prints={p['id']:meshcache[p['id']].copy().apply_transform(xform(p['actor'],p['motion'],st)) for p in D['prints']}
 for ri,r in enumerate(D['records']):
  if si==0 and ri%20==0:print('Initial clearance',ri,flush=True)
  # Fixed pins must also be checked against moving cam geometry.
  t=xform(r['actor'],r['motion'],st,r['record_id']);pts=nativecache[r['record_id']]@t[:3,:3].T+t[:3,3];lo=pts.min(0);hi=pts.max(0)
  for name,pm in prints.items():
   if r['part']=='2780.dat' and r['motion']=='fixed' and next(x for x in D['prints'] if x['id']==name)['motion']=='fixed':continue
   if np.any(np.minimum(hi,pm.bounds[1])-np.maximum(lo,pm.bounds[0])<.02):continue
   sel=pts[np.all((pts>pm.bounds[0]+.02)&(pts<pm.bounds[1]-.02),axis=1)]
   if not len(sel):continue
   pmmeta=next(x for x in D['prints'] if x['id']==name);pt=xform(pmmeta['actor'],pmmeta['motion'],st);rel=np.linalg.inv(pt)@t;key=(r['record_id'],name,np.round(rel,5).tobytes())
   if key in cache:continue
   cache.add(key);tested+=1
   # First classify inside; only measure actual interior samples.
   inside=pm.contains(sel)
   if not np.any(inside):continue
   dep=trimesh.proximity.closest_point(pm,sel[inside])[1];idx=np.argmax(dep);worst=float(dep[idx])
   if worst>(.35 if r['part']=='2780.dat' else .08):
    key=r['record_id']+' / '+name
    if key not in hits or worst>hits[key]['depth']:hits[key]=dict(depth=worst,point=sel[inside][idx].tolist(),pose=si)
 print('native state',si,'pairs',tested,'hits',len(hits),flush=True)
 (OUT/'Native clearance checks.json').write_text(json.dumps(dict(states_completed=si+1,pairs=tested,worst=hits),indent=2)+'\n')
(OUT/'Native clearance checks.json').write_text(json.dumps(dict(states=len(states),pairs=tested,tolerance_mm=.08,method='Native vertices and face centres inside printed meshes; not a complete continuous collision proof',worst=hits),indent=2)+'\n');print(json.dumps(hits,indent=2))
