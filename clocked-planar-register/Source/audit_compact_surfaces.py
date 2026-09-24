"""All-pair static triangle contact screening. Reports contacts; no blanket whitelist."""
from pathlib import Path
import json
import numpy as np
import trimesh
R=Path(__file__).resolve().parents[1]; O=R/'Compact layout'
p=json.loads((O/'parts.json').read_text());v=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
m=trimesh.collision.CollisionManager()
for x in p:
 a=v[x['offset']//3:x['offset']//3+x['vertices']]
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 m.add_object(x['id'],t)
_,names,data=m.in_collision_internal(return_names=True,return_data=True)
hits={}
for c in data:
 k=tuple(sorted(c.names));d=hits.setdefault(k,dict(a=k[0],b=k[1],depth_mm=0,contacts=0));d['depth_mm']=max(d['depth_mm'],float(c.depth));d['contacts']+=1
rows=sorted(hits.values(),key=lambda x:-x['depth_mm'])
(O/'Static surface contacts.json').write_text(json.dumps(dict(scope='All pairs, static reference mesh angles. Includes designed contacts and unresolved penetrations; not an operation pass.',pairs=rows),indent=2))
for x in rows[:65]:print(round(x['depth_mm'],4),x['a'],'/',x['b'])
print(len(rows),'contact pairs')
