"""User-directed starting arrangement: orange core rotated 180 degrees in XZ;
restore original blue carriage and remove the rejected remote lock/linkage."""
from pathlib import Path
import json,numpy as np,trimesh,shutil
R=Path(__file__).resolve().parents[1];C=R/'Core layout';O=R/'Planar register';O.mkdir(exist_ok=True)
T=trimesh.transformations.rotation_matrix(np.pi,[0,1,0],[96.8,0,16])
meta=json.loads((C/'printed-parts.json').read_text())
for p in meta:
 t=trimesh.load(C/(p['id']+'.stl'))
 if p['bank']=='Write':t.apply_transform(T)
 t.export(O/(p['id']+'.stl'),file_type='stl_ascii');p['bounds']=t.bounds.tolist()
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
hm=json.loads((C/'hardware.json').read_text());v=np.load(C/'hardware.npz')['vertices'];arr=[]
for p in hm:
 a=v[p['offset']//3:p['offset']//3+p['vertices']].copy()
 if p.get('bank')=='Write':a=trimesh.transform_points(a,T)
 p['offset']=sum(x.size for x in arr);arr.append(a)
(O/'hardware.json').write_text(json.dumps(hm,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr))
# Fresh direct inter-core static mesh contact inventory for this orientation.
a=trimesh.collision.CollisionManager();b=trimesh.collision.CollisionManager()
for p in meta:(a if p['bank']=='Memory' else b).add_object(p['id'],trimesh.load(O/(p['id']+'.stl')))
_,pairs=a.in_collision_other(b,return_names=True)
rep=dict(change='Orange write core rotated 180 degrees about Y through (96.8,0,16). Blue carriage restored from core layout. Rejected remote lock and extended arms omitted.',inter_core_static_print_contacts=[list(x) for x in sorted(pairs)],status='Reoriented starting arrangement. Lock integration and dynamic checks not yet performed.')
(O/'Orientation checks.json').write_text(json.dumps(rep,indent=2));print(json.dumps(rep,indent=2))
# Supersede old check claims; they do not apply to the reoriented layout.
for n in ['Checks.json','Hardware checks.json']:(O/n).write_text(json.dumps({'status':'SUPERSEDED: prior checks refer to the rejected extended-carriage layout. See Orientation checks.json.'},indent=2))
