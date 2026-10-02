"""Verify actual bearing faces opposing the routing-shaft retaining features.

Rigid nominal geometry only. Bush/connector grip and thrust capacity are not
inferred from their existence. Connector-coupled axle pieces are one stack.
"""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh
R=Path(__file__).resolve().parents[1]/'Compact layout';digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest();ps={p['id']:p for p in json.loads((R/'parts.json').read_text())};v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
def mesh(p):
 a=v[p['offset']//3:p['offset']//3+p['vertices']];return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
fixed=[mesh(p) for p in ps.values() if p['kind']=='printed' and p.get('motion','fixed')=='fixed']
# Negative/positive retainers and the expected opposing fixed-face stations.
checks=[
 ('Master output','master L102',-20.2,'master L069',24.4),
 ('Q output','slave L102',85.8,'slave L069',130.4),
 ('WRITE selector','write L069',-100.8,'WRITE output retainer',-104.),
 ('Master worm','Master worm retainer bearing',-24.6,'Master worm retainer gear',-31.4),
 ('Slave worm','Slave worm retainer gear',31.4,'slave_gate left clutch stub 3L stop',29.),
 ('WRITE worm','WRITE worm retainer left',-100.2,'WRITE worm retainer right',-51.8),
 ('CLOCK worm','CLOCK worm retainer',15.8,'CLK header gear 2',8.2),
 ('Master idler','master reversing idler 16T -16',-20.6,'Master idler retainer',20.6),
 ('Slave idler','slave reversing idler 16T 16',117.4,'slave reversing idler 16T 32',142.4),
 ('Selected data transfer','master_gate B-input',-48.4,'master_gate B-input',-39.6),
 ('D input','D input retainer',-100.2,'D input retainer',-95.6),
 ('D transfer','D transfer retainer',-74.4,'D transfer retainer',-69.6),
 ('Q feedback','Q feedback front gear 150',145.6,'Feedback retainer',142.4),
 ('POWER internal','master POWER 16T 32',27.4,'master POWER 16T 16',20.6),
 ('POWER incoming','POWER incoming retainer right',71.8,'POWER incoming retainer left',64.2),
 ('CLOCK incoming','CLK incoming retainer right',-16.2,'CLK incoming retainer left',-23.8)]
rows=[]
for group,neg,nface,pos,pface in checks:
 faces=[]
 for direction,name,station in [(-1,neg,nface),(1,pos,pface)]:
  p=ps[name];a=v[p['offset']//3:p['offset']//3+p['vertices']];c=(a.min(0)+a.max(0))/2
  if p.get('lego_part')=='24316':a=a[np.linalg.norm(a[:,1:]-c[1:],axis=1)>2.6]
  feature=float(a[:,0].min() if direction<0 else a[:,0].max())
  # Sample the actual annular thrust land, not a part's overall bounding box.
  found=[]
  for theta in np.linspace(0,2*np.pi,16,endpoint=False):
   origin=np.array([-200,c[1]+3.0*np.cos(theta),c[2]+3.0*np.sin(theta)])
   good=False
   for t in fixed:
    if not(t.bounds[0,0]-.01<=station<=t.bounds[1,0]+.01):continue
    points,_,tris=t.ray.intersects_location([origin],[[1,0,0]])
    if any(abs(x[0]-station)<.015 and abs(t.face_normals[j,0])>.99 for x,j in zip(points,tris)):good=True;break
   found.append(good)
  gap=(station-feature)*direction
  faces.append(dict(direction=direction,retaining_part=name,retaining_face_X_mm=feature,bearing_face_X_mm=station,gap_mm=gap,annular_samples_present=sum(found),pass_nominal=gap>=-.001 and sum(found)>=14))
 rows.append(dict(shaft_stack=group,faces=faces,nominal_end_float_mm=sum(x['gap_mm'] for x in faces),pass_nominal=all(x['pass_nominal'] for x in faces)))
out=dict(scope=__doc__,geometry_sha256=digest,stacks=rows,nominal_routing_restraint_pass=all(x['pass_nominal'] for x in rows),mechanically_qualified=False)
(R/'Routing axial restraint.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
