"""Kinematic invariants and actual phased native reaction-gear/lever checks."""
from pathlib import Path
import json,numpy as np,trimesh
from coupled_pose import pose
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';model=json.loads((O/'Coupled operation.json').read_text());parts=json.loads((O/'Coupled viewer parts.json').read_text());meta={p['id']:p for p in parts};hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];p=next(p for p in hs if p['id']=='Memory — U022');g=v[p['offset']//3:p['offset']//3+p['vertices']].reshape(-1,3,3);points=np.unique(np.concatenate([g.reshape(-1,3),g.mean(1),(g[:,0]+g[:,1])/2,(g[:,1]+g[:,2])/2,(g[:,2]+g[:,0])/2]),axis=0);lever=trimesh.load(O/'Memory — Short lever.stl');cache={};worst=0.;at=None;stationary_drive=0;overrun=0
for case in model['cases']:
 for a,b in zip(case['frames'],case['frames'][1:]):
  for bank in ['m','e']:
   dq=b['q'+bank]-a['q'+bank];dw=b['w'+bank]-a['w'+bank];dg=b['g'+bank]-a['g'+bank]
   assert abs(dq-np.pi/360*(dw-8*dg))<1e-8
   stationary_drive+=abs(dq)>1e-5 and abs(dg)<1e-7
   overrun+=abs(dq)<1e-8 and abs(dg)>1e-6
 for f in case['frames']:
  for bank in ['m','e']:
   key=(round(f['b'+bank],5),round(f['g'+bank]%45,5))
   if key not in cache:
    ff=dict(f,bm=key[0],gm=key[1]);T=np.linalg.inv(pose(meta['Memory — Short lever'],ff))@pose(meta['Memory — U022'],ff);pp=trimesh.transform_points(points,T);pp=pp[np.all((pp>lever.bounds[0])&(pp<lever.bounds[1]),axis=1)];inside=lever.contains(pp);pp=pp[inside];depth=float(trimesh.proximity.closest_point(lever,pp)[1].max()) if len(pp) else 0.;cache[key]=depth
   if cache[key]>worst:worst=cache[key];at=key
r=dict(cases=len(model['cases']),input_transitions=len(set(tuple(c['initial'][:2]+c['target']) for c in model['cases'])),frames=sum(len(c['frames']) for c in model['cases']),worm_constraint_error_mm=max(c['worm_constraint_error_mm'] for c in model['cases']),held_reaction_travel_intervals=int(stationary_drive),released_reaction_overrun_intervals=int(overrun),unique_gear_lever_poses=len(cache),maximum_gear_surface_inside_lever_mm=worst,worst_angles=at,scope='Actual viewer joint transforms; native gear surface samples against closed printed lever. Other parts and physical loads are not certified by this test.')
(O/'Coupled model checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));assert stationary_drive and overrun;assert worst<.03
