"""Rotated control cores; direct rods, without controller direction bellcranks."""
import numpy as np
import manifold3d as m

def build_controller(g):
 for key in ['P','A','OLD','LOOK','box','cyl','link','solid','triangles','inherit','emit','native']:
  globals()[key]=g[key]
 rotation=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
 fixed=m.Manifold();caps=[]
 for bank,translation in [('clock',np.array([-100.,-4.,-64.])),('write',np.array([-104.,-12.,-260.]))]:
  start=len(P)
  for old in OLD:
   n=old['id']
   keep=n.startswith(bank+' ') and not any(t in n for t in [' L0',' L1','return','frame pin','track mount'])
   if n==bank+' actuator return band':keep=True
   if bank=='clock' and n.startswith(('Clock amplifier','Clock pivot','Clock front pivot','Clock rear pivot','Rear pivot bridge','Clock output post')):keep=True
   if n in (['CLK input axle','CLOCK worm retainer','Clock track and shaft support module'] if bank=='clock' else ['WRITE input axle','WRITE worm retainer left','WRITE worm retainer right']):keep=True
   if n.startswith('Rear pivot bridge pin '):keep=False # bridge is integral with controller frame
   if keep:
    inherit(n,module='control',newname='Control '+n)
    if n=='Clock track and shaft support module':
     # Keep only the known carriage guide profile; transmission supports are new.
     track=g['oldmesh']('master carriage track')
     ss=solid(track)+box([-20,30,11.85],[20,37.8,15.9])+box([-20,30,5.3],[20,37.8,8.3])
     A[-1]=triangles(ss.rotate([0,180,0]).translate([40,0,-16]))
    if n=='Clock output post':A[-1]=triangles(solid(A[-1])^box([-200,-20,-80],[200,70,-6]))
    if bank=='write' and n in ['write Carriage fork and roof','write Right carriage bearing support']:
     A[-1]=triangles(solid(A[-1])^box([-200,-20,-80],[200,70,-20.7]))
  if bank=='clock':
   for name,c,d in [('Control CLK input 16T',[70,-5.8,-32],'CLK'),('Control CLK receiving 16T',[70,10.2,-32],'-CLK')]:native(name,'94925',c,drive=d,module='control')
   native('Control CLK external axle','3705',[70,-5.8,-32],drive='CLK',module='control')
  # Apply one rigid transform to each complete core, including its input axle.
  for i in range(start,len(P)):
   p=P[i];A[i]=A[i]@rotation.T+translation
   p['vertices']=len(A[i]);p['assembly_rotation']=rotation.tolist();p['assembly_translation']=translation.tolist()
   p['mesh_domain']='controller '+bank
   if p.get('axis') is not None:p['axis']=int(np.argmax(abs(rotation[:,p['axis']])))
  # Keep front cheeks removable; integrate rear supports into a common chassis.
  remove=[]
  for i in range(start,len(P)):
   p=P[i]
   if p['kind']=='printed' and p.get('motion','fixed')=='fixed':
    if 'Front bearing cheek' in p['id']:continue
    fixed+=solid(A[i]);remove.append(i)
  for i in reversed(remove):P.pop(i);A.pop(i)
 # Rods overlap their moving pickup blocks and form single printed components.
 for key,x,y,z0,target in [('clock',-116,10,-148,'Control Clock output post'),('write',-128,26,-212,'Control write Right carriage bearing support')]:
  rod=box([x-3,y,z0],[x+3,y+6,-48.2])+box([x-5.2,y,-60.3],[x+5.2,y+7.6,-48.2])
  rod-=cyl(2.5,y-.1,y+7.7,1,[x,0,-55])+cyl(3.3,y-.4,y+.2,1,[x,0,-55])
  if key=='clock':rod+=box([-119,15,-108],[-107,18,-100])+box([-111,17,-108],[-107,45.2,-100])
  else:rod+=box([-142,14,-176],[-138,29,-170])+box([-142,26,-176],[-126,32,-170])
  i=next(i for i,p in enumerate(P) if p['id']==target)
  joined=solid(A[i])+rod
  if len(joined.decompose())!=1:
   print(key,[(c.volume(),c.bounding_box()) for c in joined.decompose()],flush=True)
  assert len(joined.decompose())==1,'Disconnected '+key+' direct pickup'
  A[i]=triangles(joined);P[i]['vertices']=len(A[i]);P[i]['id']='Control '+key.upper()+' direct rod and pickup'
  # Loose, short guide lands: no intentional rod preload.
  for z in ([-132,-72] if key=='clock' else [-204,-140,-72]):
   guide=box([x-6,y-2,z-2],[x+6,y+8,z+2])-box([x-3.4,y-.4,z-2.1],[x+3.4,y+6.4,z+2.1])
   guide+=box([x-6,y+6.5,z-2],[x-3.5,44,z+2])
   fixed+=guide
 # Rear frame stays behind every moving part, rather than crossing the amplifier.
 frame=m.Manifold()
 for x in [-164,-108]:frame+=box([x-3,46.5,-224],[x+3,52.3,-66])
 for z in [-218,-146,-70]:frame+=box([-167,46.5,z-3],[-84,52.3,z+3])
 for component in fixed.decompose():
  b=np.array(component.bounding_box()).reshape(2,3)
  if b[1,1]<24:continue
  vv=component.to_mesh64().vert_properties[:,:3]
  rear=vv[vv[:,1]>b[1,1]-.1]
  edge=rear[rear[:,2]>rear[:,2].max()-.2];x,z=edge[:,0].mean(),edge[:,2].mean()
  frame+=box([x-3.5,b[1,1]-.4,z-3.5],[x+3.5,52.3,z+3.5])
  near=min([-218,-146,-70],key=lambda zz:abs(zz-z))
  frame+=box([x-2.5,46.5,min(z,near)-2],[x+2.5,52.3,max(z,near)+2])
 # Clearance trims apply to new ribs only; inherited bearing and pivot faces survive.
 from wall_pose import vertices,example_frames
 for p,a in zip(P,A):
  if p['module']!='control' or p['kind']!='printed' or p.get('motion','fixed')=='fixed':continue
  for f in example_frames()[::50]:
   vv=vertices(p,a,f);frame-=box(vv.min(0)-.35,vv.max(0)+.35)
 fixed+=frame
 for x,z in [(-158,-218),(-92,-146),(-158,-70)]:
  fixed+=cyl(6,46.5,52.3,1,[x,0,z]);fixed-=cyl(2.3,46.4,52.4,1,[x,0,z])
 # The direct rods translate through these clearance channels.
 for x,y,z0 in [(-116,10,-159),(-128,26,-217)]:
  fixed-=box([x-3.4,y-.4,z0],[x+3.4,y+6.4,-47])
 # Reopen only actual axle corridors. Added gear clearance at the new header.
 fixed-=cyl(9.1,-138.2,-129.8,2,[-132,-9.8,0])+cyl(9.1,-138.2,-129.8,2,[-132,6.2,0])
 for p,a in zip(P,A):
  if p['module']!='control' or p['kind']!='native':continue
  if p.get('lego_part') in ['4519','3705','3706','3707','3708','3737','60485','50450','32062','24316']:
   lo,hi=a.min(0),a.max(0);axis=p.get('axis',int(np.argmax(hi-lo)));c=(lo+hi)/2
   fixed-=cyl(2.65,lo[axis]-.1,hi[axis]+.1,axis,c)
 # Ground the guide backs and the narrow track lip around the rod corridor.
 fixed+=box([-122,43.8,-134],[-119.5,52.3,-130])+box([-164,46.5,-134],[-120,52.3,-130])
 fixed+=box([-122,43.8,-74],[-119.5,52.3,-70])
 fixed+=box([-128.1,23,-84.2],[-120,25.6,-79])+box([-123,24,-82],[-120,52.3,-79])+box([-123,46.5,-82],[-120,52.3,-69])
 # Join the new CLOCK backing past its travel end, behind the WRITE rod.
 fixed+=box([-123,32.6,-84.2],[-120,36,-79])
 # Two obsolete pin-mount pads were disconnected by the direct-rod channel.
 # They carry no shaft or working guide surface; do not retain floating sockets.
 for j,s in enumerate(fixed.decompose()):
  b=np.array(s.bounding_box()).reshape(2,3)
  obsolete=any(np.allclose(b,[[-130.4,26.2,z-3.8],[-122.8,33.8,z+3.8]],atol=.002) for z in [-116,-92])
  if obsolete:continue
  if s.volume()>1e-7:emit('control chassis '+str(j),s,module='control',color=(.3,.49,.47))
