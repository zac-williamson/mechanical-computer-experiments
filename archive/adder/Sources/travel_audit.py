from pathlib import Path
import json,sys,numpy as np,trimesh,manifold3d as m,itertools
O=Path(__file__).resolve().parents[1];D=json.loads((O/'Assembly manifest.json').read_text());sys.path.insert(0,str(O/'Sources'))
from render_ldraw import LDraw
lib=LDraw('/Applications/Studio 2.0/ldraw/parts/23948.dat')
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def overlap(a,b):
 aa=np.array(a.bounding_box());bb=np.array(b.bounding_box())
 if np.any(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])<.001):return 0
 return (a^b).volume()
fixed={};moving={}
for p in D['prints']:
 s=solid(trimesh.load(O/p['path']))
 if p['motion']=='fixed':fixed[p['id']]=s
 elif p['motion']=='carriage':moving[p['id']]=(p['actor'],s)
native={}
for r in D['records']:
 if r['part'] in ['2780.dat','32905.dat','18947.dat']:continue
 v,_=lib.mesh(r['part']);v=(v*.4@np.array(r['matrix']).reshape(3,3).T+np.array(r['pos'])*.4).reshape(-1,3)
 native[r['record_id']]=solid(trimesh.convex.convex_hull(v))
hits=[]
for n,(actor,s) in moving.items():
 for q in [-4.35,0,4.325]:
  a=s.translate([q,0,0])
  for k,b in list(fixed.items())+list(native.items()):
   vol=overlap(a,b)
   if vol>.05:hits.append([n,q,k,round(vol,3)])
for (n,(ac,a)),(k,(bc,b)) in itertools.combinations(moving.items(),2):
 if ac==bc or set([ac,bc])==set(['C','S']):continue
 for qa,qb in itertools.product([-4.35,4.325],repeat=2):
  vol=overlap(a.translate([qa,0,0]),b.translate([qb,0,0]))
  if vol>.05:hits.append([n,qa,k,qb,round(vol,3)])
(O/'Travel checks.json').write_text(json.dumps(dict(method='Carriages at both travel ends and centre against fixed prints and conservative native envelopes; pairwise independent carriage endpoints. Own clutch rings, worm meshes and pins excluded. Not full contact simulation.',hits=hits),indent=2))
print('Travel hits',len(hits));print(json.dumps(hits,indent=2))
