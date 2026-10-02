"""Independent full-layout development assembly, sourced from planar components.
All dimensions mm. Generated native parts retain actual LEGO library geometry.
"""
from pathlib import Path
import json,hashlib
import numpy as np
import trimesh
import manifold3d as m
from ldraw_mesh import LDraw
ROOT=Path(__file__).resolve().parents[1]
PLANAR=ROOT.parent/'planar-register/register-from-multiplexer/Planar register'
MUX=ROOT.parent/'planar-register/work/register-mux-reference/multiplexer'
LIB=Path('/Applications/Studio 2.0/ldraw/parts')
OUT=ROOT/'Assembly development'
OUT.mkdir(exist_ok=True)
parts=[]; provenance={}; solids={}; arrays={}

def box(a,b): return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 cc=list(c);cc[axis]=a
 return s.translate(cc)
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def mesh(s):
 a=s.to_mesh64();t=trimesh.Trimesh(np.asarray(np.round(a.vert_properties[:,:3],4),dtype=np.float32),a.tri_verts,process=True)
 if not t.is_watertight:
  t.update_faces(t.nondegenerate_faces());t.update_faces(t.unique_faces());t.remove_unreferenced_vertices()
 return t
def source(base,name):
 p=base/(name+'.stl');provenance[str(p.relative_to(ROOT.parent))]=hashlib.sha256(p.read_bytes()).hexdigest()
 return solid(trimesh.load(p))
def add(name,s,bank='frame',motion='fixed',color=None,**meta):
 s=s.simplify(.001)
 t=mesh(s)
 if not t.is_watertight:
  s=s.set_tolerance(.05).simplify(.05);t=mesh(s);meta['tessellation_cleanup_mm']=.05
 if not t.is_watertight or t.volume<=0:
  rawmesh=s.to_mesh64(); rawt=trimesh.Trimesh(rawmesh.vert_properties[:,:3],rawmesh.tri_verts,process=False)
  print('INVALID',name,s.status(),t.volume,'raw watertight',rawt.is_watertight,'components',len(rawt.split()),flush=True)
  t.export(OUT/'invalid-debug.stl')
  raise ValueError(name+' invalid solid')
 solids[name]=s;arrays[name]=t.triangles.reshape(-1,3)
 parts.append(dict(id=name,bank=bank,motion=motion,kind='printed',color=color or ([.09,.55,.61] if bank in ['master','slave'] and motion=='carriage' else [.72,.4,.19] if motion in ['rail','fork','amplifier','bellcrank'] or (bank in ['write','clock'] and motion=='carriage') else [.48,.55,.47]),**meta))
 return s
def remove_part(name):
 parts[:]=[p for p in parts if p['id']!=name]
 arrays.pop(name,None)
 return solids.pop(name,None)

raw={}
for key,base in [('planar',PLANAR),('mux',MUX)]:
 v=np.load(base/'hardware.npz')['vertices'];hs=json.loads((base/'hardware.json').read_text())
 raw[key]={h['id']:(h,v[h['offset']//3:h['offset']//3+h['vertices']].copy()) for h in hs}
def hw(name,key,source_name,delta,bank,motion=None,**meta):
 h,a=raw[key][source_name];arrays[name]=a+delta
 parts.append(dict(id=name,bank=bank,motion=motion or h['motion'],kind='native',lego_part=h.get('lego_part'),color=np.array(h.get('color',[80,90,100])).astype(float).tolist(),**meta))
 parts[-1]['color']=[x/255 for x in parts[-1]['color']]
 return arrays[name]
cache={}
def native(name,number,centre,axis,bank='frame',motion='fixed',color=(.28,.32,.35)):
 if number not in cache:
  loader=LDraw(LIB/(number+'.dat'));a,_=loader.mesh();assert not loader.missing,loader.missing
  a=a.reshape(-1,3)*.4;a-= (a.min(0)+a.max(0))/2;cache[number]=a
 a=cache[number].copy()
 # LDraw axles and gears do not share a canonical axis.
 long_axis = number in ['32073','3705','3706','3707','3708','3737','50450','50451','59443','2780','6558','4519']
 native_axis=int(np.argmax(np.ptp(a,axis=0)) if long_axis else np.argmin(np.ptp(a,axis=0)))
 a=trimesh.transform_points(a,trimesh.geometry.align_vectors(np.eye(3)[native_axis],np.eye(3)[axis]))
 arrays[name]=a+centre;parts.append(dict(id=name,bank=bank,motion=motion,kind='native',lego_part=number,color=list(color),axis=axis,centre=list(centre)))
 return arrays[name]

def memory(bank,dx):
 for p in json.loads((PLANAR/'printed-parts.json').read_text()):
  if p['id'].startswith('Memory — '):
   add(bank+' '+p['id'].split(' — ')[1],source(PLANAR,p['id']).translate([dx,0,0]),bank,p['motion'])
 for name,(h,a) in raw['planar'].items():
  if name.startswith('Memory — ') and name!='Memory — common 1 power axle' and name!='Memory — O-right 5L axle':hw(bank+' '+name.split(' — ')[1],'planar',name,[dx,0,0],bank)
  elif name.startswith('Frame pin Memory'):hw(bank+' '+name,'planar',name,[dx,0,0],bank)
 for n in ['Direct lock bolt','Detachable bolt guide']:
  add(bank+' '+n,source(PLANAR,n).translate([dx,0,0]),bank,'bolt' if n=='Direct lock bolt' else 'fixed',color=[.8,.62,.13] if n=='Direct lock bolt' else None)
 for name,(h,a) in raw['planar'].items():
  if name.startswith('Cam roller') or name.startswith('Guide mount'):hw(bank+' '+name,'planar',name,[dx,0,0],bank)
 # Preserve exact socket/guide seats, remove old orange-actuator base section.
 s=source(PLANAR,'Unified rear backbone') ^ box([-32,0,-30],[46,45,90])
 add(bank+' base',s.translate([dx,0,0]),bank)
 native(bank+' output axle','32073' if bank=='master' else '3708',[dx+(24.2 if bank=='master' else 52.2),10.2,0],0,bank,'O')

for bank,dx in [('master',0),('slave',160)]:memory(bank,dx)
# One genuine 32L axle, not overlapping coaxial copies.
native('Continuous POWER axle','50450',[80,10.2,-16],0,'power','power')

# Input clutch hardware from orange planar assembly. No redundant worm actuator.
for bank,dx in [('master_gate',0),('slave_gate',160)]:
 for name,(h,a) in raw['planar'].items():
  short=name.removeprefix('Write — ')
  if name.startswith('Write — ') and (short.startswith('L') or short.startswith('O-') or short in ['B-input','B-input-bush--10','B-input-bush--22']):
   hw(bank+' '+short,'planar',name,[dx,0,0],bank)
 hw(bank+' worm coupling','planar','2L LEGO axle joiner 59443',[dx,0,0],bank,'O')
 native(bank+' D axle','3737' if dx==0 else '3707',[-85 if dx==0 else 81.8,10.2,0],0,bank,'B')
 if dx==160:native('M transfer coupling','59443',[47,10.2,0],0,'transfer','M')
 # Lower fork halves preserve the actual planar clutch interface.
 for n in ['Carriage fork and roof','Right carriage bearing support']:
  s=source(MUX,n)^box([-30,-5,-1],[35,35,8])
  add(bank+' '+n,s.translate([-85+dx,0,16]),bank,'fork')

# The slave gate's unused clutch end needs only a 3L axle; a 5L axle
# intrudes into the independent master worm shaft.
parts[:]=[p for p in parts if p['id']!='slave_gate O-left 5L axle']
arrays.pop('slave_gate O-left 5L axle',None)
native('slave_gate O-left 3L axle','4519',[58.8,10.2,16],0,'slave_gate','O')

# WRITE feedback selector: use frozen multiplexer components, not older register.
for n in ['Front bearing cheek','Rear bearing cheek','Carriage fork and roof','Right carriage bearing support','Short lever','Left side frame','Right side frame','Left inner bearing wall','Right inner bearing wall','Common baseboard']:
 motion='carriage' if n in ['Carriage fork and roof','Right carriage bearing support'] else 'lever' if n=='Short lever' else 'fixed'
 add('write '+n,source(MUX,n).translate([-85,0,82.4]),'write',motion)
for name,(h,a) in raw['mux'].items():
 if name not in ['O-shaft','A-shaft','O-right']:hw('write '+name,'mux',name,[-85,0,82.4],'write')
# Split output axle uses planar joiner-compatible ends; replace continuous original.
for side,c in [('left',-109.2),('right',-56.8)]:native('write '+side+' output axle','32073' if side=='left' else '3706',[c,10.2,82.4],0,'write','O')
native('write feedback input axle','3708',[-33,10.2,64],0,'write','A')

# Clock actuator uses the planar actuator core, eliminating unused clutch bores.
for n in ['Front bearing cheek','Rear bearing cheek','Short lever','Carriage fork and roof','Right carriage bearing support']:
 s=source(MUX,n)
 if n in ['Carriage fork and roof','Right carriage bearing support']:s=s^box([-30,-5,8],[50,40,50])
 motion='carriage' if n in ['Carriage fork and roof','Right carriage bearing support'] else 'lever' if n=='Short lever' else 'fixed'
 add('clock '+n,s.translate([75,0,82.4]),'clock',motion)
for name,(h,a) in raw['planar'].items():
 if name.startswith('Memory — ') and any(token in name for token in ['C-shaft','selector-right','U015','U022','reaction-','pivot-','Cartridge 3L','Carriage support pin 1 18.0']):hw('clock '+name.split(' — ')[1],'planar',name,[75,0,82.4],'clock')

# Routing train: six 16T meshes preserve direction from WRITE mux to master gate.
ys=10.2-np.sqrt(16**2-(82.4/6)**2)
for i in range(7):
 z=i*82.4/6;y=10.2 if i%2==0 else ys
 native('D route gear '+str(i),'94925',[-48,y,z],0,'data_route','route')
 if i not in [0,6]:native('D route axle '+str(i),'32073',[-48,y,z],0,'data_route','route')
 for xx in [-42]:native('D route retainer '+str(i)+' '+str(xx),'32123a',[xx,y,z],0,'data_route','route')
# Four meshes return Q without reversing its logical direction.
for i in range(5):
 native('Q return gear '+str(i),'94925',[224,10.2,16*i],0,'feedback','route')
 if i not in [0,4]:native('Q return axle '+str(i),'3705',[224,10.2,16*i],0,'feedback','route')
for i in range(5):
 for x in [210,238]:native('Q return retainer '+str(i)+' '+str(x),'32123a',[x,10.2,16*i],0,'feedback','route')
native('Q feedback bus left','3708',[68,10.2,64],0,'feedback','route')
native('Q feedback bus right','50451',[186,10.2,64],0,'feedback','route')
native('Q feedback bus middle coupling','59443',[119,10.2,64],0,'feedback','route')
native('Q feedback coupling','59443',[17.5,10.2,64],0,'feedback','route')

# Bearings are constructed from actual shaft datums. Every new wall has two
# rear-facing LEGO pin joints; no unused bearing holes are added.
mounts=[]; wall_names=set()
def bearing_wall(name,x,axes,zpins,bank='frame'):
 wall_names.add(name)
 s=m.Manifold()
 for y,z in axes:
  s+=cyl(5.5,x-3.8,x+3.8,0,[0,y,z])+box([x-3.8,y,z-5.5],[x+3.8,32.4,z+5.5])
 # Narrow rear spine connects bearing bosses to both mount pins.
 lo=min([z-5.5 for y,z in axes]+[zpins[0]-4]);hi=max([z+5.5 for y,z in axes]+[zpins[-1]+4])
 s+=box([x-3.8,24.8,lo],[x+3.8,32.4,hi])
 for y,z in axes:s-=cyl(2.65,x-3.9,x+3.9,0,[0,y,z])
 for z in zpins:
  s-=cyl(2.5,24.7,32.5,1,[x,0,z])+cyl(3.35,31.7,32.5,1,[x,0,z])
  native(name+' mounting pin '+str(z),'2780',[x,32.6,z],1)
  mounts.append((x,z))
 add(name,s,bank)
 return s
# Master gate right wall also carries the intermediate data route shafts.
route_axes=[(10.2 if i%2==0 else ys,i*82.4/6) for i in range(7)]
bearing_wall('Master gate left bearing',-113,[(10.2,0),(10.2,16)],[-4,20])
bearing_wall('Master gate and route bearing',-57,[(10.2,0),(10.2,16)]+route_axes[1:6],[-4,18,56,72])
bearing_wall('Data route right bearing',-36,route_axes+[(10.2,64)],[-4,35,54,88])
bearing_wall('Slave gate clutch left bearing',49,[(10.2,16)],[8,20])
for name in ['slave_gate O-left','slave_gate L105']:arrays[name][:,0]+=2
bearing_wall('Slave gate data left bearing',59,[(10.2,0)],[-4,8])
bearing_wall('Slave gate right bearing',103,[(10.2,0),(10.2,16)],[-4,20])
for gx in [-85,75]:native('Gate outer clutch retainer '+str(gx),'32123a',[gx+34,10.2,16],0,'master_gate' if gx<0 else 'slave_gate','O')
for x in [216,232]:bearing_wall('Q return bearing '+str(x),x,[(10.2,16*i) for i in range(5)],[-4,35,54,72])
# Support feedback bus along the original front shaft plane, clear of bolt heads.
for x in [30,120]:bearing_wall('Feedback bus bearing '+str(x),x,[(10.2,64)],[64,80])
# Clock walls retain the actual planar upper cheek mounts.
for n in ['Left side frame','Right side frame']:
 s=source(PLANAR,'Memory — '+n)^box([-40,-5,8],[50,45,60])
 x=-28 if n.startswith('Left') else 28
 s+=box([x-3.8,24.8,8],[x+3.8,32.4,37.5])
 s-=cyl(2.5,22.2,32.5,1,[x,0,32])+cyl(2.5,22.2,32.5,1,[x,0,16])
 for z in [16,32]:s-=cyl(3.35,31.7,32.5,1,[x,0,z])
 s+=cyl(5,x-3.8,x+3.8,0,[0,24.2,9.6])
 s-=cyl(2.65,x-3.9,x+3.9,0,[0,24.2,9.6])
 add('clock '+n,s.translate([75,0,82.4]),'clock')
 for z in [16,32]:
  native('Clock frame pin '+str(x)+' '+str(z),'2780',[75+x,32.6,82.4+z],1)
  mounts.append((75+x,82.4+z))
native('Clock fixed guide axle','3737',[75,24.2,92],0)
for x in [41,109]:native('Clock guide axle bush '+str(x),'32123a',[x,24.2,92],0)
for partname,x0,x1 in [('clock Carriage fork and roof',59.4,82.8),('clock Right carriage bearing support',83,90.6)]:
 carriage=remove_part(partname)
 carriage+=cyl(4,x0,x1,0,[0,24.2,92])
 carriage-=cyl(2.65,x0-.1,x1+.1,0,[0,24.2,92])
 add(partname,carriage,'clock','carriage')

# WRITE port inversion: positive WRITE must select the D-side of the mux.
# One external 16T mesh reverses rotation before the original worm actuator.
for n in ['write C-shaft']:
 parts[:]=[p for p in parts if p['id']!=n];arrays.pop(n,None)
native('WRITE worm axle','3708',[-85,10.2,98.4],0,'write','input')
native('WRITE input axle','3706',[-133,10.2,114.4],0,'write_port','input')
for z in [98.4,114.4]:native('WRITE inversion gear '+str(z),'94925',[-125,10.2,z],0,'write_port','input')
for z,xs in [(98.4,[-131]),(114.4,[-143,-131,-119])]:
 for x in xs:native('WRITE inversion bush '+str(x)+' '+str(z),'32123a',[x,10.2,z],0,'write_port','input')
bearing_wall('WRITE port outer bearing',-137,[(10.2,114.4)],[106.4,122.4])
bearing_wall('WRITE port inner bearing',-113,[(10.2,114.4)],[106.4,122.4])
# Consolidate the two shared bearing datums into single printable walls.
for old,new,axes in [
 ('write Left side frame','WRITE port inner bearing',[(10.2,64),(17.9,73.2),(10.2,82.4),(10.2,98.4),(10.2,114.4)]),
 ('write Right side frame','Master gate and route bearing',[(10.2,0),(10.2,16)]+route_axes[1:6]+[(10.2,64),(17.9,73.2),(10.2,82.4),(10.2,98.4)])]:
 x=-113 if 'Left' in old else -57
 wall=remove_part(old)+remove_part(new)
 for y,z in axes:wall-=cyl(2.65,x-3.9,x+3.9,0,[0,y,z])
 for xx,z in mounts:
  if xx==x:wall-=cyl(2.5,22.2,32.5,1,[x,0,z])+cyl(3.35,31.7,32.5,1,[x,0,z])
 for z in [64,92.4]:wall-=cyl(2.5,22.2,32.5,1,[x,0,z])+cyl(3.35,31.7,32.5,1,[x,0,z])
 add(new,wall)

# Shared sequencer and positive fork drives; canonical rail coordinate is X.
from sequence import RAIL_LIMITS, bellcrank_track, RAMP_WIDTH, LIFT
from shapely.geometry import LineString

def xz(poly,y0,y1):
 return m.CrossSection([np.array(poly)],m.FillRule.EvenOdd).extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
def rounded_link(a,b,r,y0,y1):
 return (cyl(r,y0,y1,1,[a[0],0,a[1]])+cyl(r,y0,y1,1,[b[0],0,b[1]])).hull()
rail=box([-112,25.6,40],[216,31,49.7])
# Cam working width matches the native half-bush roller. A separate rear strip
# remains flat so the rail can be captured without obstructing its lobes.
for bx,sign in [(-5.05,1),(154.95,-1)]:
 crest=bx+sign*(1+RAMP_WIDTH);start=crest-sign*LIFT/1.2
 if sign>0:poly=[[start-.2/1.2,49],[bx+20,49],[bx+20,49.5],[bx+17,53.5],[crest,53.5],[start-.2/1.2,49.5]]
 else:poly=[[bx-20,49],[start+.2/1.2,49],[start+.2/1.2,49.5],[crest,53.5],[bx-17,53.5],[bx-20,49.5]]
 rail+=xz(poly,25.65,29.55)
 assert rail.status()==m.Error.NoError,('cam',bx,rail.status())
for bank,gx,stage in [('master_gate',-85,'master'),('slave_gate',75,'slave')]:
 track=[bellcrank_track(float(s),stage,gx) for s in np.linspace(*RAIL_LIMITS,501)]
 groove=LineString(track).simplify(.001).buffer(3.9,quad_segs=24)
 tab=box([gx-17,25.6,25.3],[gx+17,31,49.7])
 tab-=xz(list(groove.exterior.coords),25.5,31.1)
 rail+=tab
 rail-=xz(list(groove.exterior.coords),25.5,31.1)
 assert rail.status()==m.Error.NoError,('tab',bank,rail.status(),tab.status())
 # Planar 90-degree bellcrank, on a genuine LEGO axle.
 pivot=(gx-10,35);following=(gx,35);output=(gx-10,47)
 arm=rounded_link(pivot,following,4.8,17.8,21.2)+rounded_link(pivot,output,4.8,17.8,21.2)
 for x,z in [pivot,following,output]:arm-=cyl(2.65,17.7,21.3,1,[x,0,z])
 add(bank+' fork bellcrank',arm,bank,'bellcrank')
 native(bank+' pivot axle','4519',[gx-10,13.4,35],1,bank)
 for y in [7.8,23.4]:native(bank+' pivot bush '+str(y),'32123a',[gx-10,y,35],1,bank)
 # Existing planar reaction stop-axle geometry provides the roller axle/stop.
 hw(bank+' cam follower axle','planar','Memory — reaction-stop-axle',[gx,12,11],bank,'bellcrank')
 for y in [15.6,23.6,27.6]:native(bank+' cam follower bush '+str(y),'32123a',[gx,y,35],1,bank,'bellcrank')
 hw(bank+' output follower axle','planar','Memory — reaction-stop-axle',[gx-10,3.8,23],bank,'bellcrank')
 for y in [7.6,11.6,15.6]:native(bank+' output follower bush '+str(y),'32123a',[gx-10,y,47],1,bank,'bellcrank')
 # The pivot bridge grows from the gate's existing left bearing wall.
 wallname='Master gate left bearing' if gx<0 else 'Slave gate clutch left bearing'
 xx=-113 if gx<0 else 49
 wall=remove_part(wallname)
 wall+=box([xx-3.8,10,16],[xx+3.8,17.6,40.5])+box([xx-3.8,10,30],[gx-4.5,17.6,40.5])
 wall-=cyl(2.65,9.9,17.7,1,[gx-10,0,35])
 add(wallname,wall)
 # The drive tower belongs to the right fork half; it does not overlap the left.
 leftname=bank+' Carriage fork and roof';left=remove_part(leftname)
 rightname=bank+' Right carriage bearing support';right=remove_part(rightname)
 right+=box([gx+8.2,21.6,22],[gx+15.6,25.2,26])
 right+=box([gx+6,21.6,24.4],[gx+15.6,25.2,54.5])
 right+=box([gx-22,21.6,24.4],[gx-17.5,25.2,54.5])+box([gx-22,21.6,24.4],[gx+15.6,25.2,28])
 right+=box([gx-22,22,51.2],[gx+15.6,25.2,54.5])
 right+=box([gx-16,9.2,52.2],[gx-4,25,54.5])
 lug=box([gx-16,9.2,40.8],[gx-4,14,54.5])
 right+=lug-rounded_link((gx-10,46.3),(gx-10,47.3),3.9,9.1,14.1)
 # Two transverse joining pins, with a captured T-tail guided by the base.
 for which,x0,x1 in [('left',.25,7.7),('right',8.1,15.55)]:
  tail=box([gx+x0,27.6,15.7],[gx+x1,35.8,23.3])+box([gx+x0,33.2,14.3],[gx+x1,35.8,24.7])
  tail-=cyl(2.5,gx+x0-.1,gx+x1+.1,0,[0,32,19.5])
  if which=='left':left+=tail
  else:right+=tail
 add(leftname,left,bank,'fork');add(rightname,right,bank,'fork')
 for y in [24,32]:native(bank+' fork joining pin '+str(y),'2780',[gx+8.2,y,19.5],0,bank,'fork')
# Clear each preserved bolt-guide front lip over the entire rail stroke.
for bx in [-5.05,154.95]:rail-=box([bx-10-RAIL_LIMITS[1]-.4,25.5,45.1],[bx+10-RAIL_LIMITS[0]+.4,31.1,46.6])
# Reinforced rail joint, in the open gap beyond the slave gate.
rail+=box([102,25.6,33],[118,32.4,54])
assert rail.status()==m.Error.NoError,('joint',rail.status())
for z in [38,46]:
 rail-=cyl(2.5,101.9,118.1,0,[0,29,z])
 native('Rail joining pin '+str(z),'2780',[110,29,z],0,'clock','rail')
for name,lo,hi in [('left',-120,109.8),('right',110.2,220)]:add('Sequencer rail '+name,rail^box([lo,0,20],[hi,40,70]),'clock','rail')
# Captured guides bear on the invariant rear strip, not on the cam surface.
for x in [-45,40,134,200]:
 guide=box([x-3.8,22,31],[x+3.8,32.4,58])
 guide-=box([x-3.9,25.2,39.6],[x+3.9,31.4,50.1])
 guide-=box([x-3.9,25.2,50],[x+3.9,30,58.1])
 if x==134:guide-=box([x-4,31,36.1],[x+4,33,53.5])
 # Rear half seats against the base; the two pins pass below/above the rail.
 if x==134:guide+=box([x-3.8,22,57.8],[x+3.8,32.4,66])
 for z in ([35,62] if x==134 else [35,54]):
  guide-=cyl(2.5,24.7,32.5,1,[x,0,z])+cyl(3.35,31.7,32.5,1,[x,0,z])
  if x==134 and z==42:guide-=cyl(3.35,30.3,31.1,1,[x,0,z])+cyl(2.5,23.1,31.1,1,[x,0,z])
  native('Rail guide pin '+str(x)+' '+str(z),'2780',[x,31.2 if x==134 and z==42 else 32.6,z],1)
  mounts.append((x,z))
 add('Rail guide '+str(x),guide)


# Clock lever stays immediately behind the existing actuator, in the same thin
# Y stack as its carriage. Input/output arms are 20/60 mm on the same side.
clocklever=rounded_link((75,138),(75,78),8,28.6,32)
for z in [138,118,78]:clocklever-=cyl(2.65,28.5,32.1,1,[75,0,z])
add('Clock travel amplifier',clocklever,'clock','amplifier')
native('Clock amplifier pivot axle','3705',[75,34.4,138],1,'clock')
for y in [26.4,44.6]:native('Clock amplifier pivot bush '+str(y),'32123a',[75,y,138],1,'clock')
for label,z in [('input',118),('output',78)]:
 name='Clock '+label+' follower axle'
 hw(name,'planar','Memory — reaction-stop-axle',[75,22.6,z-24],'clock','amplifier')
 arrays[name][:,1]=60.6-arrays[name][:,1]
 arrays[name][:,2]=2*z-arrays[name][:,2]
 for y in [22.4,26.4,34.2]:native('Clock '+label+' follower bush '+str(y),'32123a',[75,y,z],1,'clock','amplifier')
# Short slot grows downward from the original roof, without changing its teeth.
name='clock Carriage fork and roof';carriage=remove_part(name)
lug=box([69,20,112],[81,28,124])
lug-=rounded_link((75,118),(75,118.5),3.9,19.9,28.1)
carriage+=lug
carriage-=rounded_link((75,118),(75,118.5),3.9,19.9,28.1)
carriage+=box([75.2,20.2,105.5],[82.8,27.8,113.7])
carriage-=cyl(2.5,75.1,82.9,0,[0,24,109.9])
add(name,carriage,'clock','carriage')
name='clock Right carriage bearing support';right=remove_part(name)
right+=box([83,20.2,105.5],[90.6,27.8,113.7])
right-=cyl(2.5,82.9,90.7,0,[0,24,109.9])
add(name,right,'clock','carriage')
native('Clock second carriage pin','2780',[83.2,24,109.9],0,'clock','carriage')
# The output slot follows only X; its vertical freedom accommodates the lever arc.
# Rear drive post: flat printable plate, three-pin attachment, roller behind
# the amplifier. Its swept window is reserved in the common frame below.
post=box([67,36.8,40],[83,42.2,85])
post-=rounded_link((75,78),(75,79.3),3.9,36.7,42.3)
name='Sequencer rail left';left=remove_part(name)
left+=box([71,25.6,49],[79,31,60])
for x,z in [(71,44),(79,44),(75,56)]:
 left+=cyl(3.8,30.9,33.4,1,[x,0,z])
 post+=cyl(3.8,33.8,36.9,1,[x,0,z])
 post-=cyl(2.5,33.7,42.3,1,[x,0,z])+cyl(3.3,33.7,34.6,1,[x,0,z])
 left-=cyl(2.5,25.5,33.5,1,[x,0,z])+cyl(3.3,32.6,33.5,1,[x,0,z])
 native('Clock drive post pin '+str(x)+' '+str(z),'2780',[x,33.6,z],1,'clock','rail')
add(name,left,'clock','rail')
add('Clock rail drive post',post,'clock','rail')
for old in ['Clock output follower axle']+['Clock output follower bush '+str(y) for y in [22.4,26.4,34.2]]:
 parts[:]=[p for p in parts if p['id']!=old];arrays.pop(old,None)
native('Clock output follower axle','3705',[75,33,78],1,'clock','amplifier')
for y in [26.4,34.2,39.5,44.4]:native('Clock output follower bush '+str(y),'32123a',[75,y,78],1,'clock','amplifier')
clocktrack=source(MUX,'Common baseboard')^box([-23.8,15.5,9],[23.8,32.8,12.2])
clocktrack=clocktrack.translate([75,0,82.4])


# The rail crosses these walls behind their bearing bosses. A forward spine
# and closely spaced mounting ears carry the load around a deliberate passage.
for name,z0,z1 in [('Master gate and route bearing',24.9,50.1),('Data route right bearing',39.6,50.1),('Q return bearing 216',39.6,50.1)]:
 wall=remove_part(name);bb=mesh(wall).bounds;x=(bb[0,0]+bb[1,0])/2
 # Route wall has an original cheek extension; its datum remains -57.
 if name=='Master gate and route bearing':x=-57
 wall+=box([x-3.8,19.2,z0-5],[x+3.8,25.2,z1+5])
 wall-=box([bb[0,0]-.1,25.2,z0],[bb[1,0]+.1,31.4,z1])
 for xx,z in mounts:
  if abs(xx-x)<.01:wall-=cyl(2.5,24.7,32.5,1,[x,0,z])+cyl(3.35,31.7,32.5,1,[x,0,z])
 add(name,wall)
# Build one frame from its shaft-support and pin datums. Only the functional
# planar carriage guide profiles are reused; no obsolete base walls are kept.
for name in ['master base','slave base','write Common baseboard']:remove_part(name)
for p in parts:
 if p['kind']=='native' and ('Frame pin' in p['id'] or 'Baseboard pin' in p['id']):
  c=(arrays[p['id']].min(0)+arrays[p['id']].max(0))/2
  mounts.append((float(c[0]),float(c[2])))
mounts=list(dict.fromkeys((round(x,5),round(z,5)) for x,z in mounts))
def rect(x0,z0,x1,z1):return m.CrossSection.square([x1-x0,z1-z0]).translate([x0,z0])
def circle(x,z,r):return m.CrossSection.circle(r,64).translate([x,z])
section=m.CrossSection()
for x0,x1,z in [(-118,237,-4),(-118,237,82),(-142,111.8,132)]:section+=rect(x0,z-4,x1,z+4)
for x in sorted(set(x for x,z in mounts)):
 zs=[z for xx,z in mounts if xx==x];near=min([-4,82,132],key=lambda zz:abs(zz-sum(zs)/len(zs)))
 section+=rect(x-3.8,min(min(zs)-4,near-4),x+3.8,max(max(zs)+4,near+4))
for x,z in mounts:section+=circle(x,z,5.2)
for x in [-113,-57,47,103]:section+=rect(x-3.8,78,x+3.8,136)
section+=rect(65,128,85,143.5)+circle(75,138,5.5)
feet=m.CrossSection()
for dx in [0,160]:
 bx=dx-5.05
 for sign in [-1,1]:
  x=bx+sign*14.5;root=dx+sign*28
  rib=m.CrossSection([np.array([[root-3.8,12],[root+3.8,12],[x+4.3,40],[x-4.3,40]])],m.FillRule.EvenOdd)
  section+=rib
  ear0,ear1=(x-6.2,x-4.6) if sign<0 else (x+4.6,x+6.2)
  foot=rect(min(x-4.3,ear0),36.2,max(x+4.3,ear1),43.8)+rect(ear0,43.7,ear1,48.5)
  for aa,bb in [(x-4.3,x-3.5),(x+3.5,x+4.3)]:foot+=rect(aa,43.7,bb,45.5)
  feet+=foot;section+=foot
 # Clear the actual detachable guide while retaining its positive seats.
 section-=rect(bx-19.1,45.5,bx+19.1,53.4)+rect(bx-10.3,53.4,bx+10.3,70.3)
for x,z in mounts:section-=circle(x,z,2.5)
section-=circle(75,138,2.65)
section-=rect(55.3,39.6,94.7,85.4)
for split in [-42,112]:
 for z in [-4,82]:section+=rect(split-8,z-5.3,split+8,z+5.3)
# Reopen pin bores after joint pads are added.
for x,z in mounts:section-=circle(x,z,2.5)
def extrude_section(cs,y0,y1):return cs.extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
back=extrude_section(section,32.8,42.4)
back+=extrude_section(feet,31.4,33)
# Retain just the planar sliding-guide surfaces and root them into this frame.
for dx,dz,base_name in [(0,0,'planar'),(160,0,'planar'),(-85,82.4,'mux'),(75,82.4,'mux')]:
 if dx==75:continue
 guide_source=source(PLANAR,'Unified rear backbone') if base_name=='planar' else source(MUX,'Common baseboard')
 track=(guide_source^box([-23.8,15.5,9],[23.8,28.2 if dx==75 else 32.8,12.2])).translate([dx,0,dz])
 back+=track
 if dx==75:
  for lo,hi in [(-28,-20),(20,28)]:back+=box([dx+lo,20,dz+9.1],[dx+hi,42.4,dz+12.1])
 else:back+=box([dx-20.8,28.4,dz+9.1],[dx+20.8,42.4,dz+12.1])+box([dx-28,32.8,dz+9.1],[dx+28,42.4,dz+12.1])
# Integral, support-free T guides restrain both fork halves in Y/Z and roll.
for gx in [-85,75]:
 g=box([gx-5,31.4,10.7],[gx+23,42.4,28.3])
 poly=[[30.5,15.3],[31.4,15.3],[32.8,13.9],[36.2,13.9],[36.2,25.1],[32.8,25.1],[31.4,23.7],[30.5,23.7]]
 hole=m.CrossSection([np.array(poly)],m.FillRule.EvenOdd).extrude(28.2).transform([[0,0,1,gx-5.1],[1,0,0,0],[0,1,0,0]])
 g-=hole
 for lo,hi in [(-5,-1.8),(19.8,23)]:g+=box([gx+lo,32.8,-4],[gx+hi,42.4,10.8])
 back+=g
# Pin collar recesses at the common wall/frame mounting plane.
for x,z in mounts:back-=cyl(2.5,32.7,42.5,1,[x,0,z])+cyl(3.35,32.7,33.5,1,[x,0,z])
back-=cyl(2.65,32.3,42.5,1,[75,0,138])
for dx in [0,160]:
 for x in [-19.55,9.45]:
  xx=x+dx;r=2.5;y=36.1
  tri=m.CrossSection([np.array([[xx-r/np.sqrt(2),y-r/np.sqrt(2)],[xx+r/np.sqrt(2),y-r/np.sqrt(2)],[xx,y-r*np.sqrt(2)]])],m.FillRule.EvenOdd).extrude(9.5).translate([0,0,36.1])
  back-=cyl(r,36.1,45.6,2,[xx,y,0])+tri
for split in [-42,112]:
 for z in [-4,82]:
  r=2.5;y=38.1
  tri=m.CrossSection([np.array([[y-r/np.sqrt(2),z-r/np.sqrt(2)],[y-r/np.sqrt(2),z+r/np.sqrt(2)],[y-r*np.sqrt(2),z]])],m.FillRule.EvenOdd).extrude(16.2).transform([[0,0,1,split-8.1],[1,0,0,0],[0,1,0,0]])
  back-=cyl(r,split-8.1,split+8.1,0,[0,y,z])+tri
  native('Base joint '+str(split)+' '+str(z),'2780',[split,y,z],0)
for name,lo,hi in [('left',-200,-42.2),('middle',-41.8,111.8),('right',112.2,270)]:
 add('Base '+name,back^box([lo,-10,-40],[hi,50,160]))

arrays['master O-left'][:,0]-=8

parts[:]=[p for p in parts if p['id']!='slave_gate O-left']
arrays.pop('slave_gate O-left',None)
# Recess the central collars of the explicitly modelled pin joints.
for p in list(parts):
 if p['kind']!='printed':continue
 name=p['id'];s=solids[name]
 for split in [-42,112]:
  if name.startswith('Base '):
   for z in [-4,82]:s-=cyl(3.3,split-1,split+1,0,[0,38.1,z])
 if name.startswith('Sequencer rail '):
  for z in [38,46]:s-=cyl(3.3,109,111,0,[0,29,z])
 if p['bank'] in ['master_gate','slave_gate'] and p['motion']=='fork':
  gx=-85 if p['bank']=='master_gate' else 75
  for y in [24,32]:s-=cyl(3.3,gx+7.1,gx+9.3,0,[0,y,19.5])
 if p['bank']=='clock' and p['motion']=='carriage':s-=cyl(3.3,82.1,84.3,0,[0,24,109.9])
 if name.startswith('Base '):
  for xx in [-19.55,9.45,140.45,169.45]:s-=cyl(3.3,42.7,44.9,2,[xx,36.1,0])
 if name=='Data route right bearing':s-=cyl(4.1,-40,-32,0,[0,10.2,16])
 if name=='Master gate and route bearing':s-=cyl(3.3,29.3,31.5,1,[-57,0,64])
 s=s.simplify(.001)
 t=mesh(s)
 if not t.is_watertight:s=s.set_tolerance(.05).simplify(.05);t=mesh(s)
 if not t.is_watertight:raise ValueError('Final mesh invalid: '+name)
 solids[name]=s;arrays[name]=t.triangles.reshape(-1,3)

# Export candidate; incomplete details remain explicit.
def export():
 for p in parts:
  if p['kind']=='printed':
   t=mesh(solids[p['id']]);positive=[c for c in t.split(only_watertight=False) if c.volume>1e-5]
   if not t.is_watertight or len(positive)!=1:raise ValueError('Invalid or disconnected print: '+p['id'])
 expected={p['id']+'.stl' for p in parts if p['kind']=='printed'}
 for stale in OUT.glob('*.stl'):
  if stale.name not in expected:stale.unlink()
 chunks=[]
 for p in parts:
  a=arrays[p['id']];p.update(offset=sum(x.size for x in chunks),vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()]);chunks.append(a)
  if p['kind']=='printed':mesh(solids[p['id']]).export(OUT/(p['id']+'.stl'))
 np.savez_compressed(OUT/'geometry.npz',vertices=np.concatenate(chunks))
 (OUT/'parts.json').write_text(json.dumps(parts,indent=2))
 (OUT/'provenance.json').write_text(json.dumps(provenance,indent=2))
 (OUT/'Status.json').write_text(json.dumps(dict(print_ready=False,working_machine_verified=False,missing=['Continuous and tolerance-aware all-part collision proof','Native keyed-axle and clutch tooth phase validation','0.1 Nm stiffness and strength qualification','Support-free print-orientation audit and print layout']),indent=2))
 print('Generated candidate',len(parts),'parts,',sum(p['kind']=='printed' for p in parts),'printed')
if __name__=='__main__':export()
