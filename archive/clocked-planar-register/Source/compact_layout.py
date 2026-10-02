"""Compact component-placement study, deliberately not a print/operation release.
Retains native core meshes; no old frame, remote rail or routing ladders.
"""
from pathlib import Path
import json, gzip, base64
import numpy as np
import trimesh
from ldraw_mesh import LDraw
R=Path(__file__).resolve().parents[1]; O=R/'Compact layout';O.mkdir(exist_ok=True)
old=json.loads((R/'Assembly development/parts.json').read_text()); raw=np.load(R/'Assembly development/geometry.npz')['vertices'].reshape(-1,3)
lookup={p['id']:p for p in old}; parts=[]; arrays=[]
from compact_keying import source_worm_phase
WORM_KEY_PHASE=source_worm_phase(lookup,raw)
def add(name,a,color,kind='native',**kw):
 a=np.asarray(a,dtype=np.float64).reshape(-1,3)
 parts.append(dict(id=name,color=color,kind=kind,offset=sum(x.size for x in arrays),vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()],**kw));arrays.append(a)
def copy(name,centre,target,flip=False):
 p=lookup[name];a=raw[p['offset']//3:p['offset']//3+p['vertices']].copy()-centre
 if flip:a*=np.array([-1,1,-1])
 a+=target;add(name,a,p['color'],p['kind'],source=name,source_centre=centre,target_centre=target,flipped=flip,motion=p.get('motion','fixed'),lego_part=p.get('lego_part'))
cache={}
def native(name,num,c,axis=0,color=(.23,.28,.31)):
 if num=='2780' and any(p.get('lego_part')==num and p.get('axis')==axis and np.allclose(p.get('centre',[1e6]*3),c,atol=1e-6) for p in parts):return
 if num not in cache:
  loader=LDraw(Path('/Applications/Studio 2.0/ldraw/parts')/(num+'.dat'));a,_=loader.mesh();assert not loader.missing
  a=a.reshape(-1,3)*.4;a-=(a.min(0)+a.max(0))/2;cache[num]=a
 a=cache[num].copy();long=num in ['3713','24316','23948','2780','44294','60485','4519','32062','32073','3705','3706','3707','3708','3737','50450','50451','59443']
 orig=int(np.argmax(np.ptp(a,axis=0)) if long else np.argmin(np.ptp(a,axis=0)))
 a=trimesh.transform_points(a,trimesh.geometry.align_vectors(np.eye(3)[orig],np.eye(3)[axis]))
 if num=='24316' and name.startswith('slave_gate left clutch stub'):
  a[:,0]*=-1;a=a.reshape(-1,3,3)[:,[0,2,1],:].reshape(-1,3)
 key_phase=0.
 if name.startswith(('master_gate worm drive','slave_gate worm drive','slave_gate left clutch stub','Master worm retainer','Slave worm retainer')):key_phase=WORM_KEY_PHASE
 elif name in ['WRITE input axle','CLK input axle','CLK header gear 2'] or name.startswith(('WRITE worm retainer','CLOCK worm retainer')):key_phase=-WORM_KEY_PHASE
 if key_phase:a=trimesh.transform_points(a,trimesh.transformations.rotation_matrix(np.radians(key_phase),np.eye(3)[axis]))
 add(name,a+c,list(color),lego_part=num,centre=c,axis=axis,key_phase_deg=key_phase)
def box(name,lo,hi,color):
 t=trimesh.creation.box(np.array(hi)-lo);t.apply_translation((np.array(lo)+hi)/2);add(name,t.triangles.reshape(-1,3),color,'envelope')
# Fixed-angle core poses only: no claim of solved tooth phasing or travel clearance.
core=['Short lever','Carriage fork and roof','Right carriage bearing support','U015','U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer','Carriage support pin 1 2.0','Carriage support pin 1 18.0']
clutch=['L072','L097','L099','L102','L069','L105']
for bank,oldx,x in [('master',0,0),('slave',160,106)]:
 for n in core+[n for n in clutch if n!='L105']:
  if bank+' '+n in lookup:copy(bank+' '+n,[oldx,0,0],[x,0,0])
 # Compact lock envelope. Not exported as fabricated printable geometry.
 box(bank+' shortened lock / guide reservation',[x-12,18,42],[x+2,35,62],[.8,.64,.18])
 # Equal magnitude +/- drive: 16/16 direct versus 16/16/16 triangle.
 for side in ([16,32] if bank=='master' else [-16,32]):native(bank+' POWER 16T '+str(side),'94925',[x+side,10.2,-16],color=(.86,.53,.12))
 # Two equal 16 mm centre distances, 16 mm between POWER and clutch axes.
 for xx in ([-16,32] if bank=='master' else [16,32]):native(bank+' reversing idler 16T '+str(xx),'94925',[x+xx,10.2+np.sqrt(192),-8],color=(.18,.46,.72))
 native(bank+' idler axle 9L','60485',[x+8,10.2+np.sqrt(192),-8])
 native(bank+' output left 3L','4519',[x-16.2,10.2,0])
 native(bank+' output right '+('6L' if bank=='master' else '7L'),'3706' if bank=='master' else '44294',[28.2 if bank=='master' else 138.2,10.2,0])
# WRITE actuator turned 180 degrees within XZ: its carriage points away from storage.
for n in core+clutch:
 if 'write '+n in lookup:copy('write '+n,[-85,0,82.4],[-76,0,-16],True)
# Dedicated WRITE reversing pair removed; D/feedback assignment is swapped.
native('WRITE input axle','60485',[-76,10.2,-32])
native('WRITE selector output left','3705',[-96.2,10.2,-16])
native('WRITE selector output right','3706',[-47.8,10.2,-16])
# Clock core below the two storage stages, without tall amplifier or remote rail.
for n in core:
 if 'master '+n in lookup:
  # Clock uses cropped native planar carriage fork; no unused output-clutch assembly.
  if n in ['Carriage fork and roof','Right carriage bearing support','Carriage support pin 1 2.0']:continue
  copy('master '+n,[0,0,0],[40,0,-28],True);parts[-1]['id']='clock '+n
for n in ['Carriage fork and roof','Right carriage bearing support']:
 copy('master '+n,[0,0,0],[40,0,-28],True);parts[-1]['id']='clock '+n
native('CLK input axle','3737',[40,10.2,-44])
# Opposite clutch sides permit a common translating crosshead. Gear offsets
# are +20/-20 (4 mm farther from neutral), with +/-7.5 mm ring travel.
for bank,oldx,x in [('master_gate',-85,-60),('slave_gate',75,60)]:
 for n in ['L097','L099']:
  copy(bank+' '+n,[oldx,0,16],[x+((-6 if bank=='master_gate' else 6) if n=='L099' else 0),0,16])
 for n in ['L102','B-input']:
  target=x if bank=='master_gate' else x-32
  copy(bank+' '+n,[oldx,0,16],[target,0,16])
 if bank=='master_gate':native(bank+' worm drive 10L','3737',[-15.8,10.2,16])
 else:
  native(bank+' worm drive 9L','60485',[100.2,10.2,16])
  native(bank+' left clutch stub 3L stop','24316',[40,10.2,16])
# One route mesh followed by the master gate mesh; invert internally.
native('Selected data route 16T','94925',[-44,10.2,-16])
native('Master gate data stub 2L','32062',[-44,10.2,0])
# WRITE D input uses one matched 8T header mesh and one 16T clutch mesh.
# Q feedback drives its free clutch gear directly, with matching net parity.
native('WRITE D input gear','94925',[-60,10.2,0],color=(.86,.53,.12))
native('WRITE D input axle 4L','3705',[-72,10.2,0])
feedback_y=10.2-np.sqrt(192)
for x in [-92,150]:native('Q feedback front gear '+str(x),'94925',[x,feedback_y,-8])
native('Q takeoff gear','94925',[150,10.2,0])
native('Q feedback shaft 32L','50450',[28,feedback_y,-8])
# POWER broken into standard lengths with coupler outside gear positions.
native('POWER shaft left 8L','3707',[14,10.2,-16]) # -20..44
native('POWER shaft joiner','59443',[47,10.2,-16])
native('POWER shaft right 12L','3708',[96,10.2,-16]) # 52..132
# Single-mesh input headers; all left edge ports X=-112.
d_y=18.2
d_z=0.
native('D header input 8T','10928',[-86,d_y,d_z])
native('D header output 8T','10928',[-86,10.2,0])
native('D external input 4L','3705',[-96,d_y,d_z])
# POWER comes in diagonally below the feedback shaft. Existing bank driver
# at X=32 is the receiving gear, avoiding a duplicate gear on the same axle.
power_y=10.2-np.sqrt(192)
native('POWER incoming left 12L','3708',[-64,power_y,-24])
native('POWER incoming joiner','59443',[-12,power_y,-24])
native('POWER incoming right 12L','3708',[40,power_y,-24])
native('POWER header gear','94925',[32,power_y,-24])
# CLK inversion is absorbed by the common fork-bar's direction assignment.
# Stop the incoming shaft at X=24 so it cannot cross the POWER header gear.
clock_raise=12.
clk_z=-37.
clk_y=10.2-np.sqrt(24**2-5**2)
native('CLK incoming left 8L','3707',[-80,clk_y,clk_z])
native('CLK incoming joiner','59443',[-44,clk_y,clk_z])
native('CLK incoming right 8L','3707',[-8,clk_y,clk_z])
native('CLK header gear 0','3648',[4,clk_y,clk_z])
native('CLK header gear 2','3648',[4,10.2,-44+clock_raise])
for part,a in zip(parts,arrays):
 if part['id'].startswith('clock ') or part['id']=='CLK input axle':
  a[:,2]+=clock_raise
  if 'target_centre' in part:part['target_centre'][2]+=clock_raise
  if 'centre' in part:part['centre'][2]+=clock_raise
 part['bounds']=[a.min(0).tolist(),a.max(0).tolist()]
# Capture routing shafts axially, using existing bearing faces wherever possible.
# Half bushes are native LEGO 4265c; input ends remain exposed.
for name,c,drive in [
 ('WRITE output retainer',[-106.2,10.2,-16],'Selected data'),
 ('Master worm retainer gear',[-35.8,10.2,16],'master worm'),
 ('Master worm retainer bearing',[-22.4,10.2,16],'master worm'),
 ('Slave worm retainer gear',[35.8,10.2,16],'slave worm'),
 ('D input retainer',[-98,d_y,d_z],'D input'),
 ('D transfer retainer',[-72,10.2,0],'D inverted'),
 ('Master idler retainer',[18.4,10.2+np.sqrt(192),-8],'master idler'),
 ('Feedback retainer',[140.2,10.2-np.sqrt(192),-8],'Q inverted'),
 ('POWER incoming retainer left',[62, power_y,-24],'POWER input'),
 ('POWER incoming retainer right',[74,power_y,-24],'POWER input'),
 ('CLK incoming retainer left',[-26,clk_y,clk_z],'CLK input'),
 ('CLK incoming retainer right',[-14,clk_y,clk_z],'CLK input'),
 ('CLOCK worm retainer',[18,10.2,-32],'clock worm'),
 ('WRITE worm retainer left',[-98,10.2,-32],'write worm'),
 ('WRITE worm retainer right',[-54,10.2,-32],'write worm')]:
 native(name,'3713' if name in ['Master worm retainer gear','Slave worm retainer gear'] else '4265c',c)
 parts[-1].update(motion='shaft-retainer',drive_group=drive)
from compact_structure import build
build(parts,arrays,add,native,lookup,raw)
from compact_elastic import actuator_band
for bank in ['master','slave','write','clock']:
 add(bank+' actuator return band',actuator_band(bank,0),[.48,.2,.52],'elastic',motion='actuator-band',bank=bank)
v=np.concatenate(arrays); lo=v.min(0);hi=v.max(0)
oldlo=np.min([p['bounds'][0] for p in old],axis=0);oldhi=np.max([p['bounds'][1] for p in old],axis=0)
size=hi-lo;baseline=oldhi-oldlo
# Measure the current physical frame and all components; motion remains unaudited.
envelope=size[[0,2]]
report=dict(status='development assembly; static dimensions only, qualification incomplete',baseline_mm=baseline.tolist(),placed_geometry_mm=size.tolist(),proposed_frame_envelope_xz_mm=envelope.tolist(),proposed_area_reduction_percent=float(100*(1-np.prod(envelope)/np.prod(baseline[[0,2]]))),worm_gears=4,clutch_rings=5,old_routing_meshes=10,new_routing_meshes=3,additional_input_header_meshes=3,power_ratio_magnitudes=[1,1],not_validated=['frame/bearings','shortened lock geometry','shared clutch sequencing linkage','moving clearances','native tooth phase','spring installation','loaded operation','port access'],bounds=[lo.tolist(),hi.tolist()])
(O/'Layout metrics.json').write_text(json.dumps(report,indent=2));(O/'parts.json').write_text(json.dumps(parts,indent=2));np.savez_compressed(O/'geometry.npz',vertices=v)
# Reuse the established local WebGL mesh viewer, but remove misleading animation.
centre=(lo+hi)/2
# Keep viewer's existing centre, translating all geometry and label points together.
delta=np.array([48,18,62])-centre
v=v+delta
ports=[['D',[-112,d_y,d_z]],['WRITE',[-112,10.2,-32]],['CLK',[-112,clk_y,clk_z]],['POWER',[-112,power_y,-24]],['Q',[166.2,10.2,0]],['MASTER',[0,28,38]],['OUTPUT',[106,28,38]],['WRITE SELECTOR',[-76,28,-48]],['CLOCK',[40,28,-60]]]
frames=[dict(joints=[[[0,0,0],[1,0,0],[0,0,0],0] for _ in parts],turns=0,Q=None,master=dict(mode='layout'),slave=dict(mode='layout'))]
data=dict(parts=parts,frames=frames,geometry=base64.b64encode(gzip.compress(v.astype('<f4').tobytes())).decode(),ports=[[n,(np.array(pt)+delta).tolist()] for n,pt in ports])
s=(R/'Source/assembly.html').read_text().replace('__DATA__',json.dumps(data,separators=(',',':')))
s=s.replace('Clocked planar register · full development assembly','Compact register · component layout')
a=s.index('<p class="note">');b=s.index('</p>',a)+4
s=s[:a]+f'<p class="note">Development assembly, not print-ready. Static complete-model envelope {envelope[0]:.1f} × {envelope[1]:.1f} mm: {report["proposed_area_reduction_percent"]:.1f}% less XZ area. Overall Y depth {size[1]:.1f} mm. Includes proposed bearing supports, frame, locks and common clutch/cam bar. Moving clearance, tooth phasing, loads and printability remain unqualified.</p>'+s[b:]
s=s.replace('<button id="play">','<button hidden id="play">').replace('<input aria-label="Capture progress"','<input hidden aria-label="Capture progress"')
a=s.index('<p>Example:');b=s.index('</p>',a)+4
s=s[:a]+'<p>Drag to orbit; scroll to zoom. Four original actuator cores, five clutch rings. The feedback shaft runs in front of the gear plane. Geometry is shown at fixed reference angles.</p>'+s[b:]
a=s.index('<p><a href="README.md">');b=s.index('</p>',a)+4
s=s[:a]+'<p><a href="Compact%20layout/Design.md">Compact design notes</a> · <a href="Viewer.html">Previous complete development assembly</a></p>'+s[b:]
s=s.replace('const sx=zoom/235','const sx=zoom/Math.max(150,82*canvas.width/canvas.height)').replace('let az=.12,el=.35','let az=0,el=0')
s=s.replace('`Input turns ${f.turns.toFixed(2)} · Q ${f.Q===null?\'undriven\':f.Q} · Master ${f.master.mode} · Output ${f.slave.mode}`','`Static placement study — no operation animation`')
s=s.replace('function draw(){',(R/'Source/compact_axes.js').read_text()+'\nfunction draw(){')
s=s.replace("document.querySelector('#readout').textContent=`Static","drawModelAxes();document.querySelector('#readout').textContent=`Static")
s=s.replace('<button id="oblique">Oblique</button>','<button id="oblique">Oblique</button><button id="depth">XY · depth</button>')
s=s.replace("window.onresize=draw;", "document.querySelector('#depth').onclick=()=>{az=Math.PI;el=Math.PI/2;draw()};window.onresize=draw;")
a=s.index('for(const [name,point]of D.ports)');b=s.index("document.querySelector('#readout')",a)
s=s[:a]+(R/'Source/compact_label_layout.js').read_text()+s[b:]
(R/'Compact layout.html').write_text(s)
print(json.dumps(report,indent=2))
