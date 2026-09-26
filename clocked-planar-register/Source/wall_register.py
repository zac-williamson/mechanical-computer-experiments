"""Build the modular planar wall-register development candidate.

Original Compact layout is read-only. Reuses actual actuator/clutch surfaces;
exports an independently identified assembly and printable development solids.
"""
from pathlib import Path
import json, hashlib, math, gzip, base64, copy
import numpy as np
import trimesh
import manifold3d as m
from ldraw_mesh import LDraw

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'Wall register'
OUT.mkdir(exist_ok=True)
BASE=ROOT/'Compact layout'
OLD=json.loads((BASE/'parts.json').read_text())
VERT=np.load(BASE/'geometry.npz')['vertices'].reshape(-1,3)
LOOK={p['id']:p for p in OLD}
P=[]; A=[]
SLAVE=np.zeros(3)
GATE=np.zeros(3)
CONTROL=np.array([0.,0.,-156.])
PITCH=112.
RODS={'clock':dict(x=-116.,y=10.,pivot_x=-96.,pivot_z=40.,radius=20.),
      'write':dict(x=-128.,y=26.,pivot_x=-104.,pivot_z=8.,radius=24.,output_radius=24.)}

def box(lo,hi):return m.Manifold.cube((np.array(hi)-lo).tolist()).translate(lo)
def cyl(r,lo,hi,axis,c):
 s=m.Manifold.cylinder(hi-lo,r,circular_segments=32)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 pos=list(c);pos[axis]=lo
 return s.translate(pos)
def link(a,b,r,y0,y1):return (cyl(r,y0,y1,1,[a[0],0,a[1]])+cyl(r,y0,y1,1,[b[0],0,b[1]])).hull()
def oldmesh(name):
 p=LOOK[name];return VERT[p['offset']//3:p['offset']//3+p['vertices']].copy()
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def triangles(s):
 for simplified in [s.simplify(.001),s.simplify(.005),s.simplify(.01),s]:
  q=simplified.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
  if not t.is_watertight:
   t.update_faces(t.unique_faces());t.update_faces(t.nondegenerate_faces(height=1e-7));t.remove_unreferenced_vertices()
  if t.is_watertight:return t.triangles.reshape(-1,3)
  for digits in [7,6,5,4]:
   t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=False);t.merge_vertices(digits_vertex=digits)
   t.update_faces(t.unique_faces());t.update_faces(t.nondegenerate_faces(height=1e-7));t.remove_unreferenced_vertices()
   if t.is_watertight:return t.triangles.reshape(-1,3)
   # Mesh export can drop a sub-micron triangular sliver. Repair only if
   # watertightness returns without a material volume change.
   t.fill_holes()
   if t.is_watertight and abs(t.volume-s.volume())<.001:return t.triangles.reshape(-1,3)
 edges,counts=np.unique(t.edges_sorted,axis=0,return_counts=True);bad=edges[counts!=2]
 print('TOPOLOGY',s.status(),len(bad),t.vertices[bad].tolist()[:20],flush=True)
 raise ValueError('Non-watertight solid volume='+str(s.volume()))

def add(name,a,kind='printed',color=(.32,.51,.49),module='bit',**meta):
 a=np.asarray(a);p=dict(id=name,kind=kind,color=list(color),module=module,vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()],**meta)
 P.append(p);A.append(a);return p

def emit(name,s,**meta):
 try:return add(name,triangles(s),**meta)
 except ValueError as e:raise ValueError(name+' '+str(e)) from e
def inherit(name,shift=(0,0,0),module='bit',newname=None):
 p=copy.deepcopy(LOOK[name]);p.pop('offset');p.pop('vertices');p.pop('bounds');p.pop('id');color=p.pop('color');kind=p.pop('kind')
 p['baseline_id']=name;p['placement_shift']=list(shift)
 return add(newname or name,oldmesh(name)+shift,kind,color,module,**p)
CACHE={}
def native(name,num,c,axis=0,drive=None,module='bit',motion='fixed',**meta):
 if num not in CACHE:
  ld=LDraw(Path('/Applications/Studio 2.0/ldraw/parts')/(num+'.dat'));v,_=ld.mesh()
  if ld.missing:raise ValueError(ld.missing)
  a=v.reshape(-1,3)*.4;a-=(a.min(0)+a.max(0))/2;CACHE[num]=a
 a=CACHE[num].copy();long=num in ['6558','3713','24316','2780','44294','60485','4519','32062','3705','3706','3707','3708','3737','50450','59443','32073']
 orig=int(np.argmax(np.ptp(a,axis=0)) if long else np.argmin(np.ptp(a,axis=0)))
 a=trimesh.transform_points(a,trimesh.geometry.align_vectors(np.eye(3)[orig],np.eye(3)[axis]))
 if name=='Control CLK receiving 16T':
  a=trimesh.transform_points(a,trimesh.transformations.rotation_matrix(math.radians(LOOK['CLK input axle'].get('key_phase_deg',0)),[1,0,0]))
 return add(name,a+c,'native',(.76,.54,.2) if num=='94925' else (.23,.28,.31),module,lego_part=num,centre=list(c),axis=axis,drive=drive,motion=motion,**meta)

# Two storage cores and the two disconnected input clutches are rigidly relocated.
# Fixed actuator working faces remain unchanged; they will be united into chassis.
for p in OLD:
 n=p['id'];bank=n.split()[0]
 if bank not in ['master','slave','master_gate','slave_gate']:continue
 if 'frame pin' in n or 'track mount' in n:continue
 if n in ['master idler axle 9L','slave idler axle 9L','slave POWER 16T 32','slave reversing idler 16T 32','slave output left 3L','slave_gate worm drive 9L','slave_gate left clutch stub 3L stop']:continue
 # Carriage pins and all working pivots remain native hardware.
 shift=GATE if bank=='slave_gate' else SLAVE if bank=='slave' else np.zeros(3)
 inherit(n,shift + (np.array([38.,0.,0.]) if n in ['master POWER 16T 32','master reversing idler 16T 32'] else 0))


native('slave_gate left clutch stub 4L','3705',[40.8,10.2,16],baseline_id='slave_gate left clutch stub 3L stop',placement_shift=[0,0,0])
native('slave output left 4L','3705',[85.8,10.2,0],drive='Q',phase_deg=7.5)
native('slave_gate worm drive 10L','3737',[104.2,10.2,16],motion='fixed',baseline_id='slave_gate worm drive 9L',placement_shift=[0,0,0])

# WRITE is now a passive selector; its actuator resides in the end module.
for n in ['write L072','write L097','write L099','write L102','write L069','write L105',
          'WRITE selector output left','WRITE output retainer',
          'WRITE D input gear',
          'Selected data route 16T','Master worm retainer gear',
          'Master idler retainer']:
 inherit(n)
native('WRITE selector output right 7L','44294',[-43.8,10.2,-16],drive='X',phase_deg=17.375)
native('WRITE D input axle 5L','32073',[-76.5,10.2,0],drive='-D',phase_deg=11.25)
native('D external input 5L','32073',[-88,2.2,0],drive='D',phase_deg=11.25)
inherit('D header input 8T',[4,-16,0])
inherit('D header output 8T',[4,0,0])
native('Master gate data stub 3L','4519',[-43.8,10.2,0],drive='-X',phase_deg=1.125)
inherit('Slave worm retainer gear',GATE)
# Reuse the original WRITE clutch finger and its joined sliding bearing block,
# cropping off the unused worm/lever tower. Its linear guide is redesigned below.
for n in ['write Carriage fork and roof','write Right carriage bearing support']:
 s=solid(oldmesh(n))^box([-115,-5,-26],[-40,30,-7])
 p=emit('Passive '+n,s,motion='write-fork')
# Preserve the original lower joining pin if it survives the crop.
inherit('write Carriage support pin 1 2.0')

# Coplanar stages restore the original straight interstage, POWER and feedback routes.
for n in ['POWER header gear','Q feedback shaft 32L','Q feedback front gear -92','Q feedback front gear 150','Q takeoff gear','Feedback retainer']:
 inherit(n,[38,0,0] if n=='POWER header gear' else [0,0,0])
native('Shared reverse shaft left 10L','3737',[12,10.2+math.sqrt(192),-8],drive='POWER',phase_deg=2.5)
native('Shared reverse shaft right 10L','3737',[94,10.2+math.sqrt(192),-8],drive='POWER',phase_deg=2.5)
native('Shared reverse coupling','59443',[53,10.2+math.sqrt(192),-8],drive='POWER',phase_deg=2.5)
native('POWER distribution 12L','3708',[56,10.2,-16],drive='-POWER',phase_deg=1.25)
native('POWER local input 4L','3705',[70,10.2-math.sqrt(192),-24],drive='POWER',phase_deg=2.5)
fy=10.2-math.sqrt(192)
# The original coplanar bar retains its working cam faces and clutch timing.
clockbar=solid(oldmesh('Common fork and cam bar'))^box([-80,-20,-80],[200,60,100])
clockbar-=box([29.9,35.9,13.1],[50.1,48,29])
clockbar+=box([-104,31,13.5],[-78,34.6,29])+box([-80,31,16],[-76,34.6,50])
clockbar-=link((-96,19.7),(-96,22.7),3.85,30.9,34.7)
assert len(clockbar.decompose())==1, 'Disconnected coplanar sequencing bar'
emit('Local clock cam and fork bar',clockbar,motion='crosshead',color=(.76,.4,.17))
for n in ['Captured crosshead guide 0','Captured crosshead guide 106']:inherit(n)
# WRITE output shoe follows the original actuator displacement, retaining the
# original fork/ring clearance rather than forcing ring engagement.
shoe=box([-120,28.4,-24],[-84,32,-8])+box([-91.6,25,-24],[-84,32,-20])
shoe-=link((-104,-16.2),(-104,-15.5),3.85,28.3,32.1)
emit('WRITE fork pickup shoe',shoe,motion='write-fork',color=(.75,.4,.18))

# Slots accommodate the circular arc: CLOCK and WRITE rod dz=-dx.
# Neither rod is asked to bend sideways as the crank rotates.
fixed_extras={'bit':m.Manifold(),'control':m.Manifold()}
for module,dz in [('bit',0)]:
 for key,c in RODS.items():
  x,z,r=c['pivot_x'],c['pivot_z']+dz,c['radius'];ro=c.get('output_radius',r);y=c['y']
  ay=18 if key=='clock' else 22.2
  aw=3.6 if key=='clock' else 3.2
  arm=link((x-r,z),(x,z),3,ay,ay+aw)+link((x,z),(x,z-ro),3,ay,ay+aw)
  for px,pz in [(x,z),(x-r,z),(x,z-ro)]:arm+=cyl(4.5,ay,ay+aw,1,[px,0,pz])
  for px,pz in [(x,z),(x-r,z),(x,z-ro)]:arm-=cyl(2.5,ay-.1,ay+aw+.1,1,[px,0,pz])
  emit(module+' '+key+' bellcrank',arm,module=module,motion='bellcrank',control_key=key,pivot=[x,0,z],radius=ro,color=(.78,.43,.2))
  # Pivot and two followers: ordinary 3L axles with opposed bushes.
  native(module+' '+key+' pivot axle','3705',[x,26 if key=='clock' else 30,z],axis=1,module=module)
  for yy in ([16,40] if key=='clock' else [19,42]):native(module+' '+key+' pivot bush '+str(yy),'32123a',[x,yy,z],axis=1,module=module)
  for label,px,pz in [('input',x-r,z),('output',x,z-ro)]:
   fc=(17 if key=='clock' else 25) if label=='input' else (25 if key=='clock' else 29)
   num='3705' if key=='clock' and label=='output' else '4519'
   native(module+' '+key+' '+label+' follower axle',num,[px,fc,pz],axis=1,module=module,motion='bellcrank',control_key=key,pivot=[x,0,z],radius=ro)
   roller_y=(13 if key=='clock' else 29) if label=='input' else (32.8 if key=='clock' else 29.9)
   native(module+' '+key+' '+label+' free roller','4265c',[px,roller_y,pz],axis=1,module=module,motion='bellcrank',control_key=key,pivot=[x,0,z],radius=ro,free_rolling=True)
   by=([7,27] if label=='input' else [16,39]) if key=='clock' else ([19,35] if label=='input' else [19,34])
   for yy in by:native(module+' '+key+' '+label+' follower bush '+str(yy),'32123a',[px,yy,pz],axis=1,module=module,motion='bellcrank',control_key=key,pivot=[x,0,z],radius=ro)
  support=cyl(5.5,ay+4,38.2,1,[x,0,z])+box([x-6,37.8,z-6],[x+6,44,z+6])
  support-=cyl(2.65,ay+3.9,44.1,1,[x,0,z])+cyl(3.8,38,44.1,1,[x,0,z])
  fixed_extras[module]+=support

# Modular rods: 112 mm pitch with a roller slot at each bit.
# Slot bridges at each bellcrank pin let the pin follow its circular X motion.
for key,c in RODS.items():
 x,y=c['x'],c['y']
 for module,z0,z1 in [('bit',-48,64)]:
  rod=box([x-3,y,z0+.2],[x+3,y+6,z1-.2])
  # Widen each splice shoe around a transverse friction-pin socket.
  for lo,hi,pinz in [(z0+.2,z0+13,z0+7),(z1-12.3,z1-.2,z1-7)]:
   rod+=box([x-5.2,y,lo],[x+5.2,y+7.6,hi])
   rod-=cyl(2.5,y-.1,y+7.7,1,[x,0,pinz])+cyl(3.3,y-.4,y+.2,1,[x,0,pinz])
  pz=c['pivot_z']+(0 if module=='bit' else -156)
  rod+=box([x-7,y,pz-6],[x+10,y+6,pz+6])
  rod-=link((x-.3,pz),(x+3,pz),3.85,y-.1,y+6.1)
  emit(module+' '+key+' vertical control rod',rod,module=module,motion='control-rod',control_key=key,color=(.8,.46,.2))
  # Opposed guides, well outside the input pickup and modular couplers.
  guide_z=([-22,21.5] if key=='clock' else [-12,32]) if module=='bit' else [-188,-88]
  for zz in guide_z:
   half=(3 if zz==-22 else 2) if module=='bit' and key=='clock' else 5
   g=box([x-6,y-2,zz-half],[x+6,y+8,zz+half])-box([x-3.4,y-.4,zz-half-.1],[x+3.4,y+6.4,zz+half+.1])
   g+=box([x-6,y+6.5,zz-half],[x-3 if key=='clock' else x+6,44,zz+half])
   fixed_extras[module]+=g
 # Front-access bridge: two pins carry rod load in shear, no clamp preload.
 s=box([x-6.6,y-8,-60.3],[x+6.6,y-.4,-35])
 for lo,hi in [(x-6.6,x-5.4),(x+5.4,x+6.6)]:
  s+=box([lo,y-.5,-60.3],[hi,y+1.5,-35])
 for zz in [-55,-41]:
  s-=cyl(2.5,y-8.1,y+1.6,1,[x,0,zz])+cyl(3.3,y-.6,y+.2,1,[x,0,zz])
 emit(key+' pinned rod splice bridge',s,module='coupler',motion='control-rod',control_key=key,color=(.64,.48,.25))
 for side,zz in enumerate([-55,-41]):
  native(key+' rod coupler friction pin '+str(side+1),'2780',[x,y-.2,zz],axis=1,module='coupler',motion='control-rod',control_key=key)

# Transmission walls are rebuilt from the actual shaft schedule after both
# modules are assembled. No legacy Bearing support solids are imported.
# Passive WRITE track: preserve guide surface around original neutral position.
track=solid(oldmesh('write carriage track'))^box([-110,15,-26],[-45,40,-20])
fixed_extras['bit']+=track+box([-100,35,-27],[-52,44,-20])

# A coordinated, open-backed chassis per module: integrate supports and retain
# narrow rails instead of separate support towers pinned to another full frame.
# Split into bed-sized development parts after all bores have been cut.
for module in ['bit']:
 fixed=fixed_extras[module]
 delete=[]
 for i,p in enumerate(P):
  if p['module']!=module or p['kind']!='printed':continue
  if p.get('motion','fixed')=='fixed':fixed+=solid(A[i]);delete.append(i)
 for i in reversed(delete):P.pop(i);A.pop(i)
 if module=='bit':
  rails=[(-28,-134,165),(40,-134,165),(56,-134,165)]
  columns=[(-132,-32,60),(-30,-32,60),(64,-32,60),(161,-32,60)]
 else:
  rails=[(-216,-146,83),(-172,-146,83),(-116,-146,83)]
  columns=[(-144,-220,-112),(-30,-220,-112),(79,-220,-112)]
 for z,x0,x1 in rails:fixed+=box([x0,38.2,z-3],[x1,44,z+3])
 for x,z0,z1 in columns:fixed+=box([x-3,38.2,z0],[x+3,44,z1])
 # Explicit load paths for the retained actuator guides and control pivots.
 # The WRITE pivot routes down to the lower rail, clear of the CLOCK follower.
 for bx in [0,106]:
  fixed+=box([bx-27.8,37.4,24.6],[bx-20.2,44,43])
  fixed+=box([bx-4,37.4,28],[bx+4,44,43])
 for zz in [-22,21.5]:fixed+=box([-132,38.2,zz-2],[-119,44,zz+2])
 fixed+=box([-107,38.2,-28],[-101,44,8])
 # Mounting eyes are unthreaded holes; frame joints use LEGO friction pins.
 mountz=[-20,52] if module=='bit' else [-208,-124]
 for x in [-126,136 if module=='bit' else 70]:
  for z in mountz:
   fixed+=cyl(6,38.2,44,1,[x,0,z]);fixed-=cyl(2.3,38.1,44.1,1,[x,0,z])
 # Full-depth slot allows the clock output post its original swept movement.
 if module=='control':
  fixed-=box([20.2,37.25,-218.4],[59.8,44.1,-126.8])
  fixed-=box([-101,28,-181],[-79,33,-176])
  fixed+=box([-120,38.2,-218],[-116,44,-183])+box([15,38.2,-218],[19,44,-201])
 if module=='bit':
  fixed-=box([-127,28,-24.4],[-76,32.4,-7.6])
  fixed-=box([-112.5,20.6,-24.5],[-95.8,25.9,18.2])
  fixed+=box([-98,38.2,-31],[-94,44,14])
  for xx in [-118,-104]:
   guide=box([xx-3,26,-27],[xx+3,38.2,-5])-box([xx-3.1,28,-24.4],[xx+3.1,32.4,-7.6])
   fixed+=guide
  fixed-=box([7.9,30.6,24.4],[32.5,35,60])
  for bx in [-5.05,100.95]:fixed-=box([bx-5,38.1,56.4],[bx+5,44.1,64.6])
 # Explicit clearance pockets for new gear sweeps and follower shafts.
 if module=='bit':
  fixed-=cyl(9.1,63.8,72.2,0,[0,10.2,0])
  fixed-=cyl(9.1,137.8,146.2,0,[0,10.2,16])+cyl(9.1,137.8,146.2,0,[0,10.2+math.sqrt(192),24])
 dz=0 if module=='bit' else -156
 fixed-=link((-108,-16+dz),(-100,-16+dz),3.95,13.8,41.2)
 fixed-=link((-106,21+dz),(-86,21+dz),3.95,22,41.2)
 # Clearance counterbores for the added thrust bushes, including axial float.
 for p,a in zip(P,A):
  if p['module']==module and 'thrust bush' in p['id']:
   lo,hi=a.min(0),a.max(0);c=(lo+hi)/2
   fixed-=cyl(3.95,lo[0]-.15,hi[0]+.15,0,[0,c[1],c[2]])
 # Make all shaft passages real even where new ribs join older geometry.
 for p,a in zip(P,A):
  if p['module']!=module or p['kind']!='native':continue
  if p.get('axis')!=0 or p.get('lego_part') not in ['4519','3705','3706','3707','3708','3737','60485','50450','32062','24316']:continue
  lo,hi=a.min(0),a.max(0);c=(lo+hi)/2
  fixed-=cyl(2.65,lo[0]-.1,hi[0]+.1,0,[0,c[1],c[2]])
 for key,c in RODS.items():
  px,pz=c['pivot_x'],c['pivot_z']+(0 if module=='bit' else -156)
  fixed-=cyl(2.65,0,50,1,[px,0,pz])+cyl(3.8,38,48,1,[px,0,pz])
  if key=='write':fixed-=cyl(3.9,16.8,21.2,1,[px,0,pz])
 if module=='bit':
  fixed+=box([-31,37.4,8],[-12,44,12])+box([118,37.4,8],[164,44,12])
 # Split the wide bit chassis at a pinned joint; rear-plane bosses do not
 # enter the local cam-bar sweep. Keep front bearing caps separately removable.
 if module=='bit':
  for zz in [-28,56]:
   fixed+=cyl(4,20.5,36.5,0,[0,44.5,zz]);fixed-=cyl(2.5,20.4,36.6,0,[0,44.5,zz])+cyl(3.3,28.1,28.9,0,[0,44.5,zz])
   native('Chassis joint pin '+str(zz),'2780',[28.5,44.5,zz],module='bit')
  # The seam is cut after all removable-wall sockets have been added.
 # All fixed solids are exported by connected component; no hidden floating islands.
 for j,s in enumerate(fixed.decompose()):
  if s.volume()<1e-7:continue
  name=module+' chassis '+str(j)
  bb=np.array(s.bounding_box()).reshape(2,3)
  if module=='bit' and bb[1,1]<10:name=('master' if bb[:,0].mean()<50 else 'slave')+' Front bearing cheek'
  emit(name,s,module=module,motion='fixed',color=(.3,.49,.47))

# Union the purposeful fork pickup joints into single fabricated parts.
for module,names in [('bit',['Passive write Right carriage bearing support','WRITE fork pickup shoe'])]:
 ids=[i for i,p in enumerate(P) if p['id'] in names];joined=m.Manifold()
 for i in ids:joined+=solid(A[i])
 for i in reversed(ids):P.pop(i);A.pop(i)
 emit(module+' WRITE fork and pickup',joined,module=module,motion='write-fork',color=(.75,.4,.18))

# Build the vertically oriented control actuators with direct rod outputs.
from wall_direct_control import build_controller
build_controller(globals())
from wall_bearing_layout import rebuild_bearings
rebuild_bearings(globals())
from wall_flat_frame import rebuild_flat_frames, reopen_control_pivots, add_fixture_seating_lands
rebuild_flat_frames(globals())
reopen_control_pivots(globals())
add_fixture_seating_lands(globals())
from wall_bed_bearings import extend_bearing_rings
extend_bearing_rings(globals())
from wall_fixture_print_faces import separate_fixture_bearing_faces
separate_fixture_bearing_faces(globals())
from wall_write_rod_print import separate_write_rod
separate_write_rod(globals())
from wall_rod_assembly import open_rod_guides
open_rod_guides(globals())
from wall_pin_seats import finish_pin_seats
finish_pin_seats(globals())

assert len({p['id'] for p in P})==len(P), 'Duplicate part IDs'
# Assign serialization offsets only after consolidation.
for p,a in zip(P,A):
 p['bounds']=[a.min(0).tolist(),a.max(0).tolist()]
offset=0
for p,a in zip(P,A):p['offset']=offset;offset+=a.size
V=np.concatenate(A)
np.savez_compressed(OUT/'geometry.npz',vertices=V)
(OUT/'parts.json').write_text(json.dumps(P,indent=2))
# Development exports are deliberately separate from a qualified print release.
stl=OUT/'Development parts';stl.mkdir(exist_ok=True)
for old in stl.glob('*.stl'):old.unlink()
for p,a in zip(P,A):
 if p['kind']=='printed':
  t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  t.export(stl/(p['id'].replace('/','-')+'.stl'))
metrics={}
for module in ['bit','control']:
 vv=np.concatenate([a for p,a in zip(P,A) if p['module']==module]);metrics[module]=dict(bounds=[vv.min(0).tolist(),vv.max(0).tolist()],static_xyz_mm=np.ptp(vv,axis=0).tolist(),parts=sum(p['module']==module for p in P))
metrics.update(row_pitch_mm=PITCH,storage_offset_mm=SLAVE.tolist(),worm_actuators_one_bit=4,worm_actuators_eight_bits=18,clock_header_teeth=[16,16],geometry_sha256=hashlib.sha256((OUT/'geometry.npz').read_bytes()).hexdigest(),status='development candidate; checks required; not a print release')
(OUT/'Layout metrics.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics,indent=2))
