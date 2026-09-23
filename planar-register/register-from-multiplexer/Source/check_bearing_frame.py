"""Check that bearings support the actual shafts and mounts agree with the base holes."""
from pathlib import Path
import json,numpy as np,trimesh
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';d=json.load(open(O/'Bearing frame datums.json'));hw=json.load(open(O/'hardware.json'));v=np.load(O/'hardware.npz')['vertices'];A={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']] for h in hw};meshes={};rows=[];theta=np.linspace(0,2*np.pi,720,endpoint=False)
for b in d['bearings']:
 n=b['part'];bank=n.split(' — ')[0];role=b['role'];shaft={'power':'common 1 power axle','data':'B-shaft','idler':'A-idler-shaft','clutch':'O-right 5L axle','worm input':'C-shaft'}[role];a=A[bank+' — '+shaft];center=(a.min(0)+a.max(0))/2;y,z=b['axis_YZ_mm'];error=float(np.linalg.norm(center[1:]-[y,z]));assert error<.001,(n,error)
 t=meshes.setdefault(n,trimesh.load(O/(n+'.stl')));lo,hi=b['x_span_mm'];lo+=.1;hi-=.1
 if b['counterbore']:
  if b['side']=='Left':lo=b['x_span_mm'][0]+4.1
  else:hi=b['x_span_mm'][1]-4.1
 for x in np.linspace(lo,hi,5):
  for r,inside in [(2.5,False),(2.9,True),(3.5,True)]:
   pts=np.c_[np.full(len(theta),x),y+r*np.cos(theta),z+r*np.sin(theta)];assert np.all(t.contains(pts)==inside),(n,role,x,r)
 rows.append(dict(b,actual_shaft=bank+' — '+shaft,axis_error_mm=error,checked_circumference_degrees=360,axial_stations=5))
base=trimesh.load(O/'Unified rear backbone.stl')
for p in d['mounts']:
 t=meshes[p['part']]
 for body,ys in [(t,[25,28.6,32.2]),(base,[33,36.6,40.2])]:
  for y in ys:
   for r,inside in [(2.3,False),(3.6 if 31.7<y<33.5 else 2.8,True)]:
    pts=np.c_[p['x']+r*np.cos(theta),np.full(len(theta),y),p['z']+r*np.sin(theta)];assert np.all(body.contains(pts)==inside),(p,y,r)
checks=dict(bearings=rows,mounts=len(d['mounts']),all_mounts_two_pins=True,scope='Shaft coaxiality, complete circumferential bearing support at five axial stations, and matching wall/base pin bores. Loads and other components require separate checks.')
(O/'Bearing support checks.json').write_text(json.dumps(checks,indent=2));print('PASS',len(rows),'bearings and',len(d['mounts']),'matching frame bores')
