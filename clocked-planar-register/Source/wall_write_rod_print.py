"""Detach the WRITE rod from its axle-bearing shoe for separate flat printing."""
import copy,json

def separate_write_rod(g):
 P,A=g['P'],g['A'];box,cyl,solid,triangles=[g[n] for n in ['box','cyl','solid','triangles']]
 name='Control WRITE direct rod and pickup';i=next(i for i,p in enumerate(P) if p['id']==name);meta=copy.deepcopy(P[i]);s=solid(A[i]);z=-171.
 bearing=s^box([-200,-30,-230],[-100,20,-40])
 rod=s^box([-200,26,-230],[-100,60,-40])
 bearing+=box([-155.8,15.6,z-4.8],[-134.8,25.6,z+4.8])
 rod+=box([-155.8,26,z-4.8],[-134.8,33.6,z+4.8])
 # Original cross-arm joins the rod to the new rear plate.
 for x in [-151.,-139.6]:
  cut=cyl(2.5,15.5,33.9,1,[x,0,z])+cyl(3.3,25.4,26.2,1,[x,0,z])
  bearing-=cut;rod-=cut
 bearing+=cyl(4.5,-176,-173.59,2,[-136,-1.8,0])-cyl(2.85,-176.01,-173.58,2,[-136,-1.8,0])
 assert len(bearing.decompose())==1 and len(rod.decompose())==1
 A[i]=triangles(rod);P[i]['vertices']=len(A[i])
 new='Control WRITE detachable bearing shoe';p=copy.deepcopy(meta);p['id']=new;p.pop('offset',None);a=triangles(bearing);p['vertices']=len(a);P.append(p);A.append(a)
 pins=[]
 for j,x in enumerate([-151.,-139.6]):
  pn='Control WRITE bearing shoe friction pin '+str(j+1);g['native'](pn,'2780',[x,25.8,z],axis=1,module='control')
  # Same translation as the original WRITE carriage, expressed in world axes.
  pp=P[-1];pp['motion']='control-rod';pp['control_key']='write'
  pins.append(dict(part=pn,centre_mm=[x,25.8,z],axis=1))
 path=g['OUT']/'Print face additions schedule.json';data=json.loads(path.read_text()) if path.exists() else {}
 data.setdefault('connections',[]).append(dict(part=new,host=name,pins=pins,print_axis=2,print_up_sign=1,bed_plane_mm=-176,assembly='Insert both pins into bearing shoe from +Y; press rod plate onto exposed halves. Attach bearing shoe to carriage before closing rod guide caps.'))
 path.write_text(json.dumps(data,indent=2))

 # A thicker frame root joined the CLOCK amplifier cradle to a rear carrier.
 # Continue its ring to the rear bed face, relieved around the existing bush.
 centre=[-156,0,-104]
 probe=cyl(3.4,39,41,1,centre)-cyl(2.9,38.9,41.1,1,centre)
 ids=[k for k,p in enumerate(P) if p['id'].startswith('control frame') and p['kind']=='printed']
 k=max(ids,key=lambda k:(solid(A[k])^probe).volume())
 foot=cyl(5.4,41.8,48.5,1,centre)-cyl(3.85,41.7,48.6,1,centre)
 A[k]=triangles(solid(A[k])+foot);P[k]['vertices']=len(A[k])
 (g['OUT']/'Fixture bed reliefs.json').write_text(json.dumps([dict(part=P[k]['id'],axle='Control Clock amplifier pivot',bed_probe_inner_mm=4.,bed_probe_outer_mm=4.9,relieved_bore_radius_mm=3.85)],indent=2))
