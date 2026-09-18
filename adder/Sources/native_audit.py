from pathlib import Path
import json,sys,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1];D=json.loads((O/'Assembly manifest.json').read_text());sys.path.insert(0,str(O/'Sources'))
from render_ldraw import LDraw
lib=LDraw('/Applications/Studio 2.0/ldraw/parts/23948.dat')
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
fixed={}
for p in D['prints']:
 if p['motion']=='fixed':fixed[p['id']]=solid(trimesh.load(O/p['path']))
hits=[]
for r in D['records']:
 if r['part']=='2780.dat':continue
 v,_=lib.mesh(r['part']);v=(v*.4@np.array(r['matrix']).reshape(3,3).T+np.array(r['pos'])*.4).reshape(-1,3)
 hull=solid(trimesh.convex.convex_hull(v));bb=np.array(hull.bounding_box())
 for n,s in fixed.items():
  bs=np.array(s.bounding_box())
  if np.any(np.minimum(bb[3:],bs[3:])-np.maximum(bb[:3],bs[:3])<.001):continue
  volume=(hull^s).volume()
  if volume>.05:hits.append([r['record_id'],n,round(volume,4)])
(O/'Native envelope checks.json').write_text(json.dumps(dict(method='Conservative native convex envelopes against fixed printed parts, zero carriage position. Pins excluded.',hits=hits),indent=2))
print('Native envelope hits',len(hits));print(json.dumps(hits,indent=2))
