# Original planar-register pin and bush meshes, with explicit new locations.
def rawhardware(n):
 p=next(p for p in hw if p['id']==n);a=hv[p['offset']//3:p['offset']//3+p['vertices']].copy()
 return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
pt=rawhardware('Memory — Carriage support pin 1 2.0');pt.apply_translation(-pt.bounds.mean(0))
def native(name,t,motion='fixed',axis=None,mates=(),kind='native',color=(.22,.25,.28)):
 parts.append(dict(name=name,mesh=t,motion=motion,kind=kind,color=list(color),axis=axis,mates=list(mates)))
def pin(name,axis,c,motion='fixed',mates=()):
 t=pt.copy();t.apply_transform(trimesh.geometry.align_vectors([1,0,0],np.eye(3)[axis]));t.apply_translation(c);native(name,t,motion,axis,mates)
for y,z in [(24,.5),(34.4,32)]:pin('Carriage joining pin '+str(z),0,[8.2,y,z],'carriage',['Carriage body','Carriage bearing end'])
for x in [-11.8,11.8]:pin('Control rod attachment pin '+str(x),1,[x,14.3,32],'carriage',['Carriage control rod','Carriage body' if x<0 else 'Carriage bearing end'])
for x in [-24.4,24.4]:
 for z in [0,50]:pin('Bearing wall pin '+str(x)+' / '+str(z),1,[x,32,z],mates=['Module base',('Left' if x<0 else 'Right')+' bearing wall'])
for z in [32,40]:pin('Actuator frame pin '+str(z),1,[36,40.4,z],mates=['Module base','Lower actuator cheek'])
for y in [31.4,39.4]:pin('Actuator cheek joining pin '+str(y),2,[34.4,y,20.2],mates=['Upper actuator cheek','Lower actuator cheek'])
for x in [-28,28]:pin('Bolt guide pin '+str(x),2,[x,42.1,39.2],mates=['Locking bolt guide','Module base'])
for original,y in [('Cam roller half bush 15.6',13.6),('Cam roller half bush 27.6',33.6)]:
 t=rawhardware(original);c=t.bounds.mean(0);t.apply_translation(np.array([-5.05,y,47.6])-c);native('Bolt follower '+('front' if y<20 else 'rear')+' bush',t,'bolt',1,['Locking bolt','Lock control rod'])
sys.path.insert(0,str(SRC.parent/'Source'))
from ldraw_mesh import LDraw
lib=LDraw('/Applications/Studio 2.0/ldraw/parts/4519.dat');a=lib.mesh()[0].reshape(-1,3)*.4;assert not lib.missing
axle=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);axle.apply_translation(-axle.bounds.mean(0));axis=int(np.argmax(axle.extents));axle.apply_transform(trimesh.geometry.align_vectors(np.eye(3)[axis],[0,1,0]));axle.apply_translation([-5.05,23.6,47.6]);native('Bolt follower axle 3L',axle,'bolt',1,['Locking bolt'])
# Band loop dimensions are inherited from the planar source viewer.
def loop(a,b,inner,outer,z0,z1):
 def cap(r):return (cy(r,z0,z1,2,[a[0],a[1],0])+cy(r,z0,z1,2,[b[0],b[1],0])).hull()
 return cap(outer)-cap(inner)
actband=loop([36.592323604,22.328448698],[22.192323604,24.828448698],2,3.2,15.4,16.6)
native('Actuator return band',mesh(actband),'actuator-band',kind='elastic',color=(.56,.25,.51))
for sign in [-1,1]:
 x=-5.05+sign*8.05
 lockband=loop([27.6,42.8],[27.6,49.6],1.5,2.1,x-.5,x+.5).rotate([90,0,90])
 native(('Left' if sign<0 else 'Right')+' lock return band',mesh(lockband),'lock-band',kind='elastic',color=(.56,.25,.51))
# Four-stud output axles retain the planar clutch's inner stop positions.
def lego_axle(number,axis,c,name):
 lib=LDraw('/Applications/Studio 2.0/ldraw/parts/'+number+'.dat');a=lib.mesh()[0].reshape(-1,3)*.4;assert not lib.missing
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);t.apply_translation(-t.bounds.mean(0));oldaxis=int(np.argmax(t.extents));t.apply_transform(trimesh.geometry.align_vectors(np.eye(3)[oldaxis],np.eye(3)[axis]));t.apply_translation(c);native(name,t,axis=axis)
for x in [-20.2,20.2]:lego_axle('3705',0,[x,10.2,0],('Left' if x<0 else 'Right')+' output axle 4L')
ret=rawhardware('Memory — selector-right-retainer');ret.apply_translation(-ret.bounds.mean(0))
for x in [-26.6,26.6]:
 t=ret.copy();t.apply_translation([x,10.2,16]);native(('Left' if x<0 else 'Right')+' input axle retainer',t,axis=0)
