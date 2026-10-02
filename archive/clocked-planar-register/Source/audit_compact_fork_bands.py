"""Native/printed surface penetration by the four fork return loops.

Samples both endpoints and maximum one-sided float. Reports actual vertices
inside solids, including all contacts: not an edge-only or force proof.
"""
from pathlib import Path
import json,hashlib,itertools
import numpy as np,trimesh
from compact_pose import vertices
from compact_elastic import fork_band
R=Path(__file__).resolve().parents[1]/'Compact layout';ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
r=json.loads((R/'Compact angle-driven operation.json').read_text());fs=[r['cases'][0]['frames'][0],r['cases'][14]['frames'][0]]
hits=[]
for bar,lag in [(-9.39,0),(-9.39,4),(9.39,0),(9.39,-4)]:
 f=json.loads(json.dumps(fs[0]));f['rail']=bar;f['clock']['q']=-bar/2.5;f['master_gate_lag']=min(lag,0);f['slave_gate_lag']=max(lag,0)
 for p in ps:
  if p.get('motion')!='fork-band':continue
  band=fork_band(p['bank'],p['fork_x'],bar,f[p['bank']+'_lag']);lo,hi=band.min(0),band.max(0)
  for q in ps:
   if q['kind'] not in ['printed','native']:continue
   vv=vertices(q,v[q['offset']//3:q['offset']//3+q['vertices']],f)
   if np.any(np.minimum(hi,vv.max(0))-np.maximum(lo,vv.min(0))<=0):continue
   t=trimesh.Trimesh(vv,np.arange(len(vv)).reshape(-1,3),process=True)
   points=np.unique(band,axis=0);inside=t.contains(points)
   if inside.any():
    closest,distance,tri=t.nearest.on_surface(points[inside]);penetrating=distance>1e-4
    if penetrating.any():hits.append(dict(band=p['id'],part=q['id'],vertices_inside=int(penetrating.sum()),max_depth_mm=float(distance.max()),bar=bar,lag=lag))
out=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),contacts=hits,sampled_band_screen_pass=not hits,mechanically_qualified=False)
(R/'Fork band clearance.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
