"""Separate the CLOCK pickup so the rod and working pickup faces print on beds."""
import copy,json

def separate_clock_pickup(g):
 P,A=g['P'],g['A'];box,cyl,solid,triangles=[g[k] for k in ['box','cyl','solid','triangles']]
 name='Control CLOCK direct rod and pickup';i=next(i for i,p in enumerate(P) if p['id']==name);meta=copy.deepcopy(P[i]);s=solid(A[i])
 rod=s^box([-200,-30,-250],[-80,36.6,0]);pickup=s^box([-200,37,-250],[-80,60,0])
 # Bed-supported arm and rising haunches: none of the rail or pickup sliding
 # faces needs a support interface. Two spaced pins transmit the pickup load.
 rod+=box([-119,10,-108],[-107,18,-100])
 for z in [-110.,-98.]:
  pad=cyl(4.8,28.8,36.6,1,[-109,0,z])
  root=box([-111,18,-108],[-107,20,-100])
  rod+=(root+cyl(4.8,28.8,29,1,[-109,0,z])).hull()+pad
  pickup+=cyl(4.8,37,45.2,1,[-109,0,z])
 from wall_flat_frame import project_back
 pickup=project_back(pickup,45.2)
 for z in [-110.,-98.]:
  bore=cyl(2.5,28.7,45.3,1,[-109,0,z])
  rod-=bore;pickup-=bore
 assert len(rod.decompose())==1 and len(pickup.decompose())==1
 A[i]=triangles(rod);P[i]['vertices']=len(A[i])
 new='Control CLOCK detachable pickup';p=copy.deepcopy(meta);p['id']=new;p.pop('offset',None);a=triangles(pickup);p['vertices']=len(a);P.append(p);A.append(a)
 pins=[]
 for j,z in enumerate([-110.,-98.]):
  pn='Control CLOCK pickup friction pin '+str(j+1);g['native'](pn,'2780',[-109,36.8,z],axis=1,module='control')
  P[-1].update(motion='control-rod',control_key='clock');pins.append(dict(part=pn,centre_mm=[-109,36.8,z],axis=1))
 path=g['OUT']/'Print face additions schedule.json';data=json.loads(path.read_text())
 data['connections'].append(dict(part=new,host=name,pins=pins,print_axis=1,print_up_sign=-1,bed_plane_mm=45.2,assembly='Print the pickup rear-face down and rod front-face down. Seat both 2780 pins in the rod from +Y, then press the pickup onto the exposed halves before installing the rod and actuator.'))
 path.write_text(json.dumps(data,indent=2))
