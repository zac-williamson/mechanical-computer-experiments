"""Functional register cartridges. Every cassette owns its gates, timing and bearings.

Standalone and combined scenes use the same part records; joining never replaces
an internal axle, support, gear or cam. Geometry work runs only under run_wall_job.
"""
from pathlib import Path
import json, copy, hashlib, math, heapq
import numpy as np
import trimesh
import manifold3d as m
from shapely.geometry import Point, LineString, MultiPoint
from shapely.ops import unary_union
from wall_pose import vertices as posed
from wall_print_geometry import qualify
from ldraw_mesh import LDraw
R=Path(__file__).resolve().parents[1]; OLD=R/'Wall register'; O=R/'Functional register';O.mkdir(exist_ok=True)
L={p['id']:p for p in json.load(open(OLD/'parts.json'))};V=np.load(OLD/'geometry.npz')['vertices'].reshape(-1,3)
P=[];A=[];S={};PRINT={};MOUNTS={};BORES=[]
def box(lo,hi):return m.Manifold.cube(np.array(hi)-lo).translate(lo)
def cyl(r,lo,hi,axis,c):
 s=m.Manifold.cylinder(hi-lo,r,circular_segments=32)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=lo;return s.translate(c)
def mesh(s):
 for ss in [s.simplify(.001),s.simplify(.005),s.simplify(.01),s]:
  q=ss.to_mesh64()
  for digits in [None,7,6,5,4]:
   t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
   if digits is not None:t.merge_vertices(digits_vertex=digits)
   t.update_faces(t.unique_faces());t.update_faces(t.nondegenerate_faces(height=1e-7));t.remove_unreferenced_vertices()
   if t.is_watertight:return t
   t.fill_holes()
   if t.is_watertight and abs(t.volume-s.volume())<.001:return t
 edges,counts=np.unique(t.edges_sorted,axis=0,return_counts=True);bad=edges[counts!=2]
 print('FAILED TOPOLOGY',s.status(),s.volume(),len(bad),t.vertices[bad].tolist()[:18],flush=True)
 raise ValueError('Export topology failed')

def sol(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def add(n,a,owner,kind='printed',source=None,orient=None,**meta):
 p=copy.deepcopy(source) if source else {};p.update(id=n,kind=kind,module='bit',owner=owner,**meta);p.pop('offset',None)
 P.append(p);A.append(np.asarray(a));
 if orient:PRINT[n]=orient
 return p

def emit(n,s,owner,orient,**meta):
 assert len([c for c in s.decompose() if c.volume()>.001])==1,(n,'disconnected',[(c.volume(),c.bounding_box()) for c in s.decompose()])
 S[n]=s;return add(n,mesh(s).triangles.reshape(-1,3),owner,orient=orient,**meta)
def take(n,owner,new=None,shift=(0,0,0)):
 p=L[n];a=V[p['offset']//3:p['offset']//3+p['vertices']].copy()
 q=add(new or n,a+shift,owner,p['kind'],p)
 if np.any(shift):q['assembly_rotation']=np.eye(3).tolist();q['assembly_translation']=list(shift)
 return q
CACHE={}
def native(n,num,c,owner,axis=0,drive=None):
 if num not in CACHE:
  ld=LDraw(Path('/Applications/Studio 2.0/ldraw/parts')/(num+'.dat'));v,_=ld.mesh();assert not ld.missing
  a=v.reshape(-1,3)*.4;a-=(a.min(0)+a.max(0))/2;CACHE[num]=a
 a=CACHE[num].copy();long=num in ['2780','6558','4519','3705','32073','3706','44294','3707','3737','3708','59443']
 ax=int(np.argmax(np.ptp(a,axis=0)) if long else np.argmin(np.ptp(a,axis=0)))
 a=trimesh.transform_points(a,trimesh.geometry.align_vectors(np.eye(3)[ax],np.eye(3)[axis]))+c
 return add(n,a,owner,'native',lego_part=num,axis=axis,centre=list(c),drive=drive,motion='fixed')

# A first print is a COMPLETE powered gated latch, not a slide/rail coupon.
# The active dog clutch, compliant fork, lock cam and storage output stay together.
for bank in ['master','slave']:
 owner=bank;dx=0 if bank=='master' else 32
 for n,p in L.items():
  if p['module']!='bit' or not (n.startswith(bank+' ') or n.startswith(bank+'_gate ')):continue
  if any(w in n for w in ['POWER','reversing idler']):continue
  # Slave B-input is the previous stage's output gear, hence is a port owned
  # by the master; a dedicated slave input stub is added below.
  if n=='slave_gate B-input':continue
  take(n,owner,shift=[dx,0,0])
 for n in (['Master worm retainer gear','Master gate data stub 3L','Master worm axial stop -26.2','M output axial stop 37.8','Master data stub axial stop -53.8','Master data stub axial stop -34.2'] if bank=='master' else ['Slave worm retainer gear','Slave worm axial stop 128.2','Slave worm axial stop 135.4','Q output axial stop 76.2','Q output axial stop 83.8']):take(n,owner,shift=[dx+(.4 if n=='Slave worm axial stop 135.4' else 0),0,0])
 # Retain cheek carrier and both of its pins; it belongs to this cassette.
 host='bit frame 0 removable fixture 4' if bank=='master' else 'bit frame 1 removable fixture 0'
 schedule=next(e for e in json.load(open(OLD/'Frame fixture schedule.json'))['fixtures'] if e['part']==host)
 take(host,owner,new=bank+' cheek carrier',shift=[dx,0,0])
 for pin in schedule['fasteners']:take(pin['part'],owner,shift=[dx,0,0])
 mounts=[]
 for e in json.load(open(OLD/'Frame fixture schedule.json'))['fixtures']:
  if e['part'].startswith(bank+' ') or e['part']==host:
   mounts += [np.array(p['centre_mm'])+[dx,0,0] for p in e['fasteners']]
 MOUNTS[owner]=mounts
 # Power direction pair is LOCAL; no shared shaft or shared bearing through a cassette.
 off=0 if bank=='master' else 106+dx
 minus=16 if bank=='master' else -16;plus=-16 if bank=='master' else 16
 header=off+32
 for label,x,y,z in [('lower clutch supply',off+minus,10.2,-16),('upper clutch supply',off+plus,10.2+math.sqrt(192),-8),('inverter lower',header,10.2,-16),('inverter upper',header,10.2+math.sqrt(192),-8),('power input gear',header,10.2-math.sqrt(192),-24)]:native(bank+' '+label,'94925',[x,y,z],owner,drive='POWER' if 'upper' in label or 'input' in label else '-POWER')
 if bank=='master':
  native('master lower supply axle','3706',[28,10.2,-16],owner,drive='-POWER') # 4..52
  native('master upper supply axle','3737',[12,10.2+math.sqrt(192),-8],owner,drive='POWER') # -28..52
  native('master power port axle','3705',[32,10.2-math.sqrt(192),-24],owner,drive='POWER') #16..48
  native('master data port axle','44294',[-59.8,10.2,-16],owner,drive='X')
  take('Selected data route 16T',owner)
  native('master output inboard stop','4265c',[30.2,10.2,0],owner,drive='M')
 else:
  native('slave lower supply axle','3737',[off+8,10.2,-16],owner,drive='-POWER') # off-32..off+48
  native('slave upper supply axle','3737',[off+8,10.2+math.sqrt(192),-8],owner,drive='POWER')
  native('slave power port axle','32073',[off+30,10.2-math.sqrt(192),-24],owner,drive='POWER') # off+8..off+48
  native('slave data port axle','32073',[74.2,10.2,0],owner,drive='M')
  take('slave_gate B-input',owner,new='slave data port gear',shift=[32,0,0])
  # Output port already has its own pair of bearings inside the slave frame.
  take('Q takeoff gear',owner,new='slave feedback output gear',shift=[32,0,0])
 # Use the exact inherited cam and floating-fork slots. Local cam portions
 # retain their local stroke and lock/dog-clutch sequencing.
 p=L['Local clock cam and fork bar'];a=V[p['offset']//3:p['offset']//3+p['vertices']]
 s=sol(a)^box([-80 if bank=='master' else 30,-60,-100],[29.6 if bank=='master' else 155,60,100])
 # A straight rear extension supplies two shear-pin sockets for a detachable
 # clock link, outside cam/fork running surfaces. Print from its rear Y face.
 # Interface geometry is assessed after the unmodified working faces below.
 # Raise a detachable connection above the working cam; the connection
 # travels with the cam and clears both stationary guide blocks throughout.
 terminal=60 if bank=='master' else 70
 s=s.translate([dx,0,0])
 if bank=='slave':s=s^box([65.2,-100,-100],[300,100,150])
 if bank=='master':s+=box([28.8,30,46.6],[64.8,32.2,54])
 s+=box([terminal-4.8,25.6,46.6],[terminal+4.8,32.6,76.8])
 for zz in [60,72]:s-=cyl(2.5,24.7,32.7,1,[terminal,0,zz])+cyl(3.3,31.9,32.7,1,[terminal,0,zz])
 emit(bank+' local timing bar',s,owner,(1,-1),motion='crosshead',inherited_cam=True)
 # These guides belong to the latch, so the cam is captured during a bench
 # test even without the other latch or the shared control module.
 for cx,sign in [((-30.6 if bank=='master' else 107.4),-1),((34 if bank=='master' else 172),1)]:
  xlo=cx-1.6;xhi=cx+1.6
  g=box([xlo,22.2,42.4],[xhi,40.3,76.8])-box([xlo-.1,25.2,46.2],[xhi+.1,32.6,54.4])
  g+=box([xlo,25.2,46.1],[xhi,29.8,46.6])
  if bank=='slave' and sign==-1:
   # The cam's front underside is flat at Z47. Support it on a broad
   # Z46.6 land (0.4 clearance). Open the unused rear underside over the
   # cheek carrier; no thin roof or load-bearing flap remains there.
   g-=box([xlo-.1,29.8,42.3],[xhi+.1,40.4,46.2])
   g+=box([xlo,25.2,46.1],[xhi,29.8,46.6])
  # Broad upper heel, clear of the cam's vertical connecting webs.
  bed=xlo if sign==1 else xhi;far=bed+sign*10.1
  g+=box([min(bed,far),32.7,55.2],[max(bed,far),40.3,76.8])
  px=bed+sign*3.8
  for zz in [60,72]:
   def cut(r,lo,hi):
    d=r/math.sqrt(2);pts=[[px+sign*d,zz-d],[px+sign*d,zz+d],[px+sign*r*math.sqrt(2),zz]]
    return cyl(r,lo,hi,1,[px,0,zz])+m.CrossSection([pts],m.FillRule.EvenOdd).extrude(hi-lo).transform([[1,0,0,0],[0,0,1,lo],[0,1,0,0]])
   g-=cut(2.5,32.4,40.4);g-=cut(3.3,39.6,40.4)
   native(bank+' timing guide '+str(cx)+' pin '+str(zz),'2780',[px,40.5,zz],owner,axis=1)
   MOUNTS[owner].append(np.array([px,40.5,zz]))
  emit(bank+' timing guide '+str(cx),g,owner,(0,sign))

# One removable clock link joins two already-captured timing bars. Four pins
# transmit the force in shear, without changing either timing cam.
link=box([55.2,33,55.2],[74.8,40.6,76.8])
for xx in [60,70]:
 for zz in [60,72]:
  link-=cyl(2.5,32.9,40.7,1,[xx,0,zz])+cyl(3.3,32.9,33.7,1,[xx,0,zz])
  native('Timing connection pin '+str(xx)+' '+str(zz),'2780',[xx,32.8,zz],'connections',axis=1)
  P[-1]['motion']='crosshead'
emit('Removable timing connection',link,'connections',(1,-1),motion='crosshead')

# Removable, phase-preserving external axle couplings. No internal axle is
# withdrawn or replaced when the cassettes are docked.
native('M interstage coupling','59443',[53.2,10.2,0],'connections',drive='M')
native('POWER connection shaft','3708',[98,10.2-math.sqrt(192),-24],'connections',drive='POWER')
for x in [49,147]:native('POWER coupling '+str(x),'59443',[x,10.2-math.sqrt(192),-24],'connections',drive='POWER')
# Retention is local to each cassette; mating connectors are never end stops.
for owner,y,z,xs,label in [
 ('master',10.2,-16,[-71.8,-64.2],'data'),
 ('master',10.2,-16,[42.2,49.8],'minus'),
 ('master',10.2+math.sqrt(192),-8,[42.2,49.8],'plus'),
 ('master',10.2-math.sqrt(192),-24,[18.2,25.8],'input'),
 ('slave',10.2,-16,[108.2,115.8],'minus'),
 ('slave',10.2+math.sqrt(192),-8,[108.2,115.8],'plus'),
 ('slave',10.2-math.sqrt(192),-24,[178.2,185.8],'input'),
 ('slave',10.2,0,[85.6],'data')]:
 for x in xs:native(owner+' '+label+' port stop '+str(x),'4265c',[x,y,z],owner)

from functional_timing_parts import split_timing
split_timing(globals())

# Seat the full 1.6 mm collar of each carriage joining pin. The right half
# prints in -X from its outer X=15.6 face (153.6 for the slave). Both axle
# bores remain vertical; the socket opens toward the top of the print.
for bank,dx in [('master',0),('slave',138)]:
 n=bank+' Right carriage bearing support';i=next(i for i,p in enumerate(P) if p['id']==n)
 # Replace the obsolete offset counterbore region with the original full
 # joining boss, then cut one continuous socket at the actual raised pin.
 ss=sol(A[i])+box([dx+8,20.2,0],[dx+15.6,27.8,8.7])

 if bank=='slave':ss-=cyl(9.25,dx+7.9,dx+15.7,0,[0,10.2+math.sqrt(192),-8])
 ss-=cyl(3.3,dx+7.9,dx+9.4,0,[0,24,5.3])+cyl(2.5,dx+7.9,dx+15.7,0,[0,24,5.3])
 taper=m.Manifold.cylinder(.9,3.3,2.5,circular_segments=32).rotate([0,90,0]).translate([dx+9.4,24,5.3])
 ss-=taper
 raw=ss.to_mesh64()
 tt=trimesh.Trimesh(raw.vert_properties[:,:3],raw.tri_verts,process=True)
 assert tt.is_watertight,n
 A[i]=tt.triangles.reshape(-1,3);PRINT[n]=(0,-1);S[n]=ss


# Reopen the full pin insertion paths in the retained fixtures. Their earlier
# bed-face extensions had left 0.2/0.4 mm caps beyond the drill extents.
for bank,dx in [('master',0),('slave',138)]:
 for side,px,sign in [('left',-20.5,1),('right',10.5,-1)]:
  n=bank+' bolt guide '+side;i=next(i for i,p in enumerate(P) if p['id']==n);ss=sol(A[i])
  for zz in [49,59]:
   xx=px+dx;r=2.5;d=r/math.sqrt(2)
   roof=m.CrossSection([[[xx+sign*d,zz-d],[xx+sign*d,zz+d],[xx+sign*r*math.sqrt(2),zz]]],m.FillRule.EvenOdd).extrude(8.4).transform([[1,0,0,0],[0,0,1,32.4],[0,1,0,0]])
   ss-=cyl(r,32.4,40.8,1,[xx,0,zz])+roof
  A[i]=mesh(ss).triangles.reshape(-1,3);PRINT[n]=(0,sign);S[n]=ss
 n=bank+' removable rear bearing cheek';i=next(i for i,p in enumerate(P) if p['id']==n);ss=sol(A[i])
 for zz in [28.4,35.6]:ss-=cyl(2.5,1.7,25.9,1,[dx-24,0,zz])
 # Retain the elastic's seat/centre. Ramp the retaining head in the -Y
 # print direction, keeping the groove at Y9.25..11.2 unobstructed.
 ax,az=dx+31.192323604,29.128448698
 # Replace the whole head/stem to avoid remnants of the old triangulated
 # lip. A full rear root reaches the bed instead of sprouting from a recess.
 ss-=box([ax-3.4,7,az-3.4],[ax+3.4,11.2001,az+3.4])
 ss+=cyl(3.25,11.2,18.60001,1,[ax,0,az])+cyl(2,9.2,11.21,1,[ax,0,az])
 ss+=cyl(3.2,7.2,7.9,1,[ax,0,az])
 ss+=m.Manifold.cylinder(1.4,3.2,1.9,circular_segments=32).rotate([-90,0,0]).translate([ax,7.9,az])
 A[i]=mesh(ss).triangles.reshape(-1,3);PRINT[n]=(1,-1);S[n]=ss

# Save only once the complete geometry has been generated.
def save():
 off=0
 for p,a in zip(P,A):p.update(offset=off,vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()]);off+=a.size
 (O/'parts.json').write_text(json.dumps(P,indent=2));np.savez_compressed(O/'geometry.npz',vertices=np.concatenate(A))
print('Extracted complete mechanism cartridges',len(P),flush=True)

# Bearings own exactly the shafts in their cassette. Constant-X sections put
# all axle bores on the bed; only the two pin heels thicken toward print +X.
CASES=json.load(open(R/'Compact layout/Compact contact-resolved operation.json'))['cases']
FS=[]
for ca in CASES:
 for i in np.linspace(0,len(ca['frames'])-1,9,dtype=int):FS.append(ca['frames'][i])
SCENE=[]
for p,a in zip(P,A):
 if p['kind']=='elastic':continue
 seen=set();vs=[]
 for f in FS:
  v=posed(p,a,f);key=tuple(np.round(np.r_[v.min(0),v.max(0)],4))
  if key not in seen:seen.add(key);vs.append(v)
 SCENE.append((p,a,vs))
print('Pose envelopes collected',len(SCENE),flush=True)

def crossshape(v,x,half=1.65):
 tri=v.reshape(-1,3,3);tri=tri[(tri[:,:,0].min(1)<x+half)&(tri[:,:,0].max(1)>x-half)]
 if not len(tri):return None
 edges=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]);d=edges[:,1,0]-edges[:,0,0];edges=edges[abs(d)>1e-8];d=edges[:,1,0]-edges[:,0,0]
 flat=tri.reshape(-1,3);pts=[flat[(flat[:,0]>=x-half)&(flat[:,0]<=x+half)]]
 for xx in [x-half,x+half]:
  u=(xx-edges[:,0,0])/d;ok=(u>=0)&(u<=1);pts.append(edges[ok,0]+u[ok,None]*(edges[ok,1]-edges[ok,0]))
 pts=np.concatenate(pts)
 return MultiPoint(pts[:,1:]).convex_hull if len(pts) else None

def extrude_yz(sh,x0,thick):
 polys=list(sh.geoms) if sh.geom_type=='MultiPolygon' else [sh];result=m.Manifold()
 for pp in polys:
  if pp.is_empty or pp.area<1e-5:continue
  co=[np.array(pp.exterior.coords)[:-1]]+[np.array(h.coords)[:-1] for h in pp.interiors]
  result+=m.CrossSection(co,m.FillRule.EvenOdd).extrude(thick).transform([[0,0,1,x0],[1,0,0,0],[0,1,0,0]])
 return result

def tear_y(x,z,r,lo=32.4,hi=40.4,sign=1):
 d=r/math.sqrt(2);pts=[[x+sign*d,z-d],[x+sign*d,z+d],[x+sign*r*math.sqrt(2),z]]
 return cyl(r,lo,hi,1,[x,0,z])+m.CrossSection([pts],m.FillRule.EvenOdd).extrude(hi-lo).transform([[1,0,0,0],[0,0,1,lo],[0,1,0,0]])

schedule=[
 ('master','output',['master output left 3L','master output right 6L'],10.2,0,[-24,34]),
 ('master','worm',['master_gate worm drive 10L'],10.2,16,[-30,22]),
 ('master','data idler',['Master gate data stub 3L'],10.2,0,[-50,-38]),
 ('master','data port',['master data port axle'],10.2,-16,[-68,-38]),
 ('master','lower supply',['master lower supply axle'],10.2,-16,[10,46]),
 ('master','upper supply',['master upper supply axle'],10.2+math.sqrt(192),-8,[-24,46]),
 ('master','power port',['master power port axle'],10.2-math.sqrt(192),-24,[22,38]),
 ('slave','output',['slave output left 4L','slave output right 7L'],10.2,0,[112,164]),
 ('slave','worm',['slave_gate left clutch stub 4L','slave_gate worm drive 10L'],10.2,16,[60.8,164]),
 ('slave','data port',['slave data port axle'],10.2,0,[64.2,81.8]),
 ('slave','lower supply',['slave lower supply axle'],10.2,-16,[112,182]),
 ('slave','upper supply',['slave upper supply axle'],10.2+math.sqrt(192),-8,[112,182]),
 ('slave','power port',['slave power port axle'],10.2-math.sqrt(192),-24,[158,182])]
GROUP={}
for owner,label,names,y,z,stations in schedule:
 for x in stations:GROUP.setdefault((owner,x),[]).append((label,names,y,z))
FEET=[]
for (owner,x),entries in GROUP.items():
 sign=-1 if x in [-38,60.8] else 1
 foot_lo=x-1.6 if sign==1 else x-8.5;foot_hi=x+8.5 if sign==1 else x+1.6
 foot=next(z for z in [-28,-52,-76] if all(abs(z-zz)>20 or foot_hi+.4<lo or foot_lo-.4>hi for o,lo,hi,zz in FEET))
 FEET.append((owner,foot_lo,foot_hi,foot))
 shapes=[];sn=[];ignored={n for e in entries for n in e[1]}
 for p,a,vs in SCENE:
  if p['id'] in ignored:continue
  local=[]
  for v in vs:
   if v[:,0].min()>=x+1.65 or v[:,0].max()<=x-1.65:continue
   sh=crossshape(v,x)
   if sh is not None:local.append(sh)
  if local:shapes.append(unary_union(local));sn.append(p['id'])
 obstacle=unary_union(shapes).buffer(.35,resolution=5);forbid=obstacle.buffer(2.6,resolution=5)
 profile=Point(39,foot).buffer(2.5,resolution=8)
 for label,names,y,z in entries:
  ring=Point(y,z).buffer(5,resolution=16).difference(Point(y,z).buffer(2.65,resolution=16))
  hits=[(n,ring.intersection(sh).area) for n,sh in zip(sn,shapes) if ring.intersection(sh).area>.01]
  if hits:print('RING OBSTACLE',owner,label,x,hits,flush=True);raise ValueError('Bearing station not clear')
  start=np.array([y,z]);goal=np.array([39.,float(foot)]);dist={(0,0):0};prev={};q=[(0.,(0,0))];end=None
  def pos(k):return start+2*np.array(k)
  while q:
   _,k=heapq.heappop(q);a=pos(k)
   if not LineString([a,goal]).intersects(forbid):end=k;break
   for du,dv in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
    kk=(k[0]+du,k[1]+dv);b=pos(kk)
    if not (-20<=b[0]<=41 and -90<=b[1]<=66):continue
    dd=dist[k]+2*math.hypot(du,dv)
    if dd>=dist.get(kk,1e9) or LineString([a,b]).intersects(forbid):continue
    dist[kk]=dd;prev[kk]=k;heapq.heappush(q,(dd+np.linalg.norm(b-goal),kk))
  if end is None:raise ValueError(('No load path',owner,x,label))
  keys=[end]
  while keys[-1]!=(0,0):keys.append(prev[keys[-1]])
  raw=[pos(k) for k in keys[::-1]]+[goal];path=[raw[0]];j=0
  while j<len(raw)-1:
   jj=next(h for h in range(len(raw)-1,j,-1) if not LineString([raw[j],raw[h]]).intersects(forbid));path.append(raw[jj]);j=jj
  profile=unary_union([profile,Point(y,z).buffer(5,resolution=16),LineString(path).buffer(2.5,resolution=8)])
  BORES.append(dict(owner=owner,shaft=label,parts=names,axis='X',station=x,y=y,z=z,land=3.2))
 # Broad triangular plate where it clears the moving envelopes.
 broad=profile.convex_hull.difference(obstacle);profile=unary_union([profile,broad])
 for label,names,y,z in entries:profile=profile.difference(Point(y,z).buffer(2.65,resolution=16))
 s=extrude_yz(profile,x-1.6,3.2)
 # The mounting foot overlaps the constant-X plate at the bed face.
 s=s^box([-300,-100,-100],[300,40.3,150])
 px=x+sign*2.2;lo=x-1.6 if sign==1 else x-8.5;hi=x+8.5 if sign==1 else x+1.6
 s+=box([lo,32.7,foot-9.8],[hi,40.3,foot+9.8])
 for zz in [foot-6,foot+6]:
  s-=tear_y(px,zz,2.5,sign=sign);s-=tear_y(px,zz,3.3,39.6,40.4,sign=sign)
  native(owner+' wall '+str(x)+' pin '+str(zz),'2780',[px,40.5,zz],owner,axis=1)
  MOUNTS[owner].append(np.array([px,40.5,zz]))
 n=owner+' bearing plate '+str(x);emit(n,s,owner,(0,sign));print('Bearing',n,flush=True)

for owner in ['master','slave']:
 mounts=MOUNTS[owner];xmin=min(c[0] for c in mounts)-6;xmax=max(c[0] for c in mounts)+6
 xmin=xmin if owner=='master' else 53.4;xmax=53 if owner=='master' else xmax
 zmin=min(c[2] for c in mounts)-6;zmax=max(65,max(c[2] for c in mounts)+6)
 frame=box([xmin,45.5,zmin],[xmin+5,48.5,zmax])+box([xmax-5,45.5,zmin],[xmax,48.5,zmax])
 for z in sorted(set([-37,-22,0,30,40,49,62,zmin+3]+[float(c[2]) for c in mounts])):frame+=box([xmin,45.5,z-3],[xmax,48.5,z+3])
 for c in mounts:
  x,y,z=c;frame+=box([x-4.8,y+.2,z-4.8],[x+4.8,48.5,z+4.8])
 for c in mounts:
  x,y,z=c;frame-=cyl(2.5,y+.19,48.6,1,c)+cyl(3.3,y+.19,y+.9,1,c)
 # A pair of horizontal friction pins keys the two complete chassis at
 # the final datum. The two frames are already separate on the print bed.
 for zz in [-5,40]:
  lo=45.2 if owner=='master' else 53.4;hi=53 if owner=='master' else 61.2
  frame+=box([lo,37.8,zz-4.8],[hi,48.5,zz+4.8])
  d=2.5/math.sqrt(2)
  roof=m.CrossSection([[[44.5-d,zz-d],[44.5-d,zz+d],[44.5-2.5*math.sqrt(2),zz]]],m.FillRule.EvenOdd).extrude(16.2).transform([[0,0,1,45.1],[1,0,0,0],[0,1,0,0]])
  frame-=cyl(2.5,45.1,61.3,0,[0,44.5,zz])+roof
  # A 2780 collar spans 1.6 mm; the seam gap is only 0.4 mm. Relieve both
  # halves, with the same 45-degree roof used for the horizontal pin bore.
  r=3.3;d=r/math.sqrt(2)
  reliefroof=m.CrossSection([[[44.5-d,zz-d],[44.5-d,zz+d],[44.5-r*math.sqrt(2),zz]]],m.FillRule.EvenOdd).extrude(2.0).transform([[0,0,1,52.2],[1,0,0,0],[0,1,0,0]])
  frame-=cyl(r,52.2,54.2,0,[0,44.5,zz])+reliefroof
 # Apply the seam last; mounting additions cannot silently rejoin the frames.
 frame=frame^box([-300 if owner=='master' else 53.4,-100,-150],[53 if owner=='master' else 300,100,150])
 emit(owner+' cassette chassis',frame,owner,(1,-1))
for zz in [-5,40]:native('Cassette frame joint '+str(zz),'2780',[53.2,44.5,zz],'connections',axis=0)
# Preserve the actual inherited keyed-shaft phases, including both ends of
# each removable coupling. New gear placement does not create a free phase.
for p in P:
 if p['kind']!='native' or 'baseline_id' in p:continue
 d=p.get('drive');phase={'-POWER':1.25,'POWER':2.5,'X':17.375,'M':1.125}.get(d)
 if phase is not None:p['phase_deg']=phase
save()
(O/'Architecture.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256((OLD/'geometry.npz').read_bytes()).hexdigest(),bearing_schedule=BORES,print_orientations=PRINT,mounts={k:[x.tolist() for x in v] for k,v in MOUNTS.items()},state='development; interfaces and validation pending'),indent=2))
print('Cartridge candidate saved',len(P),flush=True)
