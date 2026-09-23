"""Bounded timing search, not a global mechanical minimum or a force solution."""
from pathlib import Path
import json,itertools,numpy as np
from shapely.geometry import LineString
from shapely import points,covers
R=Path(__file__).resolve().parents[1];out=[];tested=0
angles=np.radians(np.linspace(-50,50,5001));xx=10-10*np.cos(angles);zz=10*np.sin(angles)
for radius,op,cl,start,end in itertools.product([20,25,30,35,40,45,50],[-13,-14,-15,-16],[3,4,5,6],[.6,.9,1.2],[2.1,2.4,2.7]):
 tested+=1
 qs=np.r_[np.linspace(-8,start,20),np.linspace(start,end,80),np.linspace(end,8,20)];cmd=op+(cl-op)*np.clip((qs-start)/(end-start),0,1);u=(5.8+cmd)/radius
 if np.max(np.abs(u))>=1:continue
 path=LineString(np.c_[10-10*np.sqrt(1-u*u)+qs,10*u]);free=path.buffer(3.8,quad_segs=24).buffer(-3.6,quad_segs=24)
 ranges=[]
 for q in [-.2,3.35]:
  vals=[]
  for dx in [-.6,.6]:vals.extend((radius*np.sin(angles)-5.8)[covers(free,points(xx+q+dx,zz))].tolist())
  ranges.append((min(vals),max(vals)) if vals else (None,None))
 if ranges[0][1] is not None and ranges[0][1]<=-9.6 and ranges[1][0] is not None and ranges[1][0]>=.35:
  out.append(dict(output_radius_mm=radius,input_radius_mm=10,open_command_mm=op,close_command_mm=cl,cam_start_q=start,cam_end_q=end,reconnect_max_command_mm=ranges[0][1],hold_min_command_mm=ranges[1][0]))
rep=dict(candidates_tested=tested,feasible_count=len(out),smallest_output_radius_mm=min((x['output_radius_mm'] for x in out),default=None),feasible=sorted(out,key=lambda x:(x['output_radius_mm'],abs(x['open_command_mm']),x['close_command_mm'])),limitations=['Only timing at the monotone boundary poses and selected error corners.','Does not check available material around groove, follower reactions, bearing deflection, or installation.','Finite grid, not proof of smallest possible register.'])
(R/'Investigation/Compact cam search.json').write_text(json.dumps(rep,indent=2));print(json.dumps({k:v for k,v in rep.items() if k!='feasible'},indent=2));print(rep['feasible'][:3])
