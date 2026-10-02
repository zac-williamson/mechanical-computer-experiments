"""Native mesh surface intersections in three phase-aware operation poses.

Contacts include intentional shaft/gear interfaces. FCL triangle contact depth
is not a reliable solid penetration measure; findings require interface review.
"""
import json,hashlib
from pathlib import Path
import numpy as np,trimesh
from compact_pose import transform
R=Path(__file__).resolve().parents[1]/'Compact layout';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
p=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
cases=json.loads((R/'Compact contact-resolved operation.json').read_text())['cases'];frames=next(c['frames'] for c in cases if c['start']==[1,1,0] and c['end']==[1,1,1] and c['initial_Q']==0)
manager=trimesh.collision.CollisionManager();native=[]
for part in p:
 if part['kind']!='native':continue
 a=v[part['offset']//3:part['offset']//3+part['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);manager.add_object(part['id'],t);native.append(part)
hits={}
for fi in [0,150,500]:
 f=frames[fi]
 for part in native:manager.set_transform(part['id'],transform(part,f))
 coll,names,data=manager.in_collision_internal(return_names=True,return_data=True)
 for a,b in names:hits.setdefault(tuple(sorted((a,b))),[]).append(fi)
result=dict(scope=__doc__,geometry_sha256=digest,poses=[0,150,500],surface_contacts=[dict(a=a,b=b,frames=fs) for (a,b),fs in sorted(hits.items())],mechanically_qualified=False)
(R/'Native pair surface screening.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
