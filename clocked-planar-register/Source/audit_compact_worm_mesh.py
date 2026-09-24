"""Exhaustive one-degree assembly-phase screen of inherited worm/8T meshes.

For each sampled reaction-wheel angle at either carriage end, try every worm
phase. Failure to find a phase means these meshes cannot certify that pose;
it does not alone prove that the real LEGO parts bind. No mesh is trimmed.
"""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh
R=Path(__file__).resolve().parents[1]/'Compact layout';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest();ps={p['id']:p for p in json.loads((R/'parts.json').read_text())};v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
cm=trimesh.collision.CollisionManager();names=['master U015','master U022']
for n in names:
 p=ps[n];a=v[p['offset']//3:p['offset']//3+p['vertices']];cm.add_object(n,trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True))
rows=[]
for q in [-3.755874,3.749041]:
 for g in range(45):
  cm.set_transform(names[1],trimesh.transformations.rotation_matrix(np.radians(g),[0,1,0],[0,10.2,24]));clear=[]
  for w in range(360):
   t=trimesh.transformations.rotation_matrix(np.radians(w),[1,0,0],[0,10.2,16]);t[0,3]+=q;cm.set_transform(names[0],t)
   if not cm.in_collision_internal():clear.append(w)
  rows.append(dict(carriage_mm=q,reaction_angle_deg=g,clear_worm_phases_deg=clear))
 print(q,'no fit at',sum(not x['clear_worm_phases_deg'] for x in rows if x['carriage_mm']==q),'reaction poses',flush=True)
out=dict(scope=__doc__,geometry_sha256=digest,tested_phase_combinations=len(rows)*360,poses_without_any_clear_phase=[x for x in rows if not x['clear_worm_phases_deg']],poses=rows,sampled_worm_mesh_pass=all(x['clear_worm_phases_deg'] for x in rows),mechanically_qualified=False)
(R/'Inherited worm mesh screening.json').write_text(json.dumps(out,indent=2));print('Phase combinations',out['tested_phase_combinations'],'unresolved poses',len(out['poses_without_any_clear_phase']))
