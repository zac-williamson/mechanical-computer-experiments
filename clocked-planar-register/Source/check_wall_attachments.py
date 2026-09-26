"""Audit fixed-component attachment topology; not a strength qualification."""
import manifold3d as m
from check_wall_supports import P,mesh,solid,fixed,cyl,O,np,json,hashlib
pairs=[('master Front bearing cheek','master'),('slave Front bearing cheek','slave'),('Control clock Front bearing cheek','Control clock'),('Control write Front bearing cheek','Control write')]
checks=[]
for name,prefix in pairs:
 cheek=next(p for p in P if p['id']==name)
 pins=[p for p in P if p['id'].startswith(prefix+' cheek joining pin ')]
 centres=[np.array(p['bounds']).mean(0) for p in pins]
 # Both pin axes must engage the removable cheek and the integral rear cheek.
 bores=[]
 for pin,c in zip(pins,centres):
  lo,hi=np.array(pin['bounds']);a=solid(mesh(cheek));b=fixed[cheek['module']]-a
  front=cyl(3.4,lo[1]+.5,lo[1]+6.5,1,c)-cyl(2.7,lo[1]+.4,lo[1]+6.6,1,c)
  rear=cyl(3.4,hi[1]-6.5,hi[1]-.5,1,c)-cyl(2.7,hi[1]-6.6,hi[1]-.4,1,c)
  ratios=[(front^a).volume()/front.volume(),(rear^b).volume()/rear.volume()]
  bores.append(dict(pin=pin['id'],socket_wall_fraction=ratios,pass_check=min(ratios)>.95))
 spacing=float(np.linalg.norm(centres[0]-centres[1])) if len(centres)==2 else 0
 checks.append(dict(component=name,attachment='two spaced pin axes',spacing_mm=spacing,sockets=bores,pass_check=len(pins)==2 and spacing>=7 and all(x['pass_check'] for x in bores)))
schedule=json.loads((O/'Bearing schedule.json').read_text())
wallchecks=[]
for entry in schedule['removable_walls']:
 p=next(p for p in P if p['id']==entry['part']);wall=solid(mesh(p))
 rear=sum((solid(mesh(p)) for p in P if p['id'].startswith(entry['module']+' coordinated chassis ')),m.Manifold())
 sockets=[]
 for pin in entry['pins']:
  c=np.array(pin['centre_mm']);y=c[1]
  front=cyl(3.4,y-7.5,y-1.5,1,c)-cyl(2.7,y-7.6,y-1.4,1,c)
  back=cyl(3.4,y+1.5,y+7.5,1,c)-cyl(2.7,y+1.4,y+7.6,1,c)
  fractions=[(front^wall).volume()/front.volume(),(back^rear).volume()/back.volume()]
  sockets.append(dict(pin=pin['part'],socket_wall_fraction=fractions,pass_check=min(fractions)>.95))
 wallchecks.append(dict(**entry,sockets=sockets,pass_check=len(sockets)>=2 and entry['pin_spacing_mm']>=10 and all(x['pass_check'] for x in sockets)))
# Separate guide/actuator fixtures use paired LEGO 2780 friction pins.
fixturechecks=[]
for entry in json.loads((O/'Frame fixture schedule.json').read_text())['fixtures']:
 fp=next(p for p in P if p['id']==entry['part']);ss=solid(mesh(fp))
 bp=next(p for p in P if p['id']==entry['frame']);bs=solid(mesh(bp));back=np.array(bp['bounds'])[1,1]
 checks_mount=[]
 for bolt in entry['fasteners']:
  c=np.array(bolt['centre_mm']);y=c[1]
  shell=cyl(3.4,y-7.5,y-1.5,1,c)-cyl(2.7,y-7.6,y-1.4,1,c)
  frame_shell=cyl(3.4,y+1.5,y+7.5,1,c)-cyl(2.7,y+1.4,y+7.6,1,c)
  seat=y+.2
  land_front=cyl(3.7,seat-.1,seat,1,c)-cyl(3.4,seat-.2,seat+.1,1,c)
  land_back=cyl(3.7,seat,seat+.1,1,c)-cyl(3.4,seat-.1,seat+.2,1,c)
  frac=[(shell^ss).volume()/shell.volume(),(frame_shell^bs).volume()/frame_shell.volume(),(land_front^ss).volume()/land_front.volume(),(land_back^bs).volume()/land_back.volume()]
  pin=next(p for p in P if p['id']==bolt['part'])
  checks_mount.append(dict(part=bolt['part'],socket_wall_fraction=frac,pass_check=min(frac)>.95 and pin.get('lego_part')=='2780'))
 fixturechecks.append(dict(**entry,mount_checks=checks_mount,pass_check=len(checks_mount)>=2 and entry['fastener_spacing_mm']>=6 and all(a['pass_check'] for a in checks_mount)))
allowed={x['part'] for x in fixturechecks}|{name for name,_ in pairs}|{x['part'] for x in schedule['removable_walls']}|{'bit coordinated chassis 0','bit coordinated chassis 1','control coordinated chassis 0'}
from wall_extra_connections import extra_connections
extra=extra_connections(O)
allowed|={e['part'] for e in extra}|{e['host'] for e in extra}
revision=json.loads((O/'Revision connection checks.json').read_text()) if extra else {'connection_pass':True}
if extra:assert revision['geometry_sha256']==hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
unassigned=[p['id'] for p in P if p['kind']=='printed' and p.get('motion','fixed')=='fixed' and p['module'] in ['bit','control'] and p['id'] not in allowed]
# Two actual frame solids, an open seam, and two engaged horizontal pins.
frame_parts=[p for p in P if p['id'].startswith('bit coordinated chassis ')]
frame_solids=[solid(mesh(p)) for p in frame_parts]
frame_union=sum(frame_solids,m.Manifold())
seam=m.Manifold.cube([.38,65,600]).translate([28.31,35,-400])
seam_volume=(frame_union^seam).volume()
frame_pins=[]
for z in [-28,56]:
 pin=next(p for p in P if p['id']=='Chassis joint pin '+str(z));c=np.array(pin['bounds']).mean(0)
 sides=[]
 for lo,hi in [(21,27.5),(29.5,36)]:
  ring=cyl(3.4,lo,hi,0,c)-cyl(2.7,lo-.1,hi+.1,0,c)
  # Probe the circular seating sides below the intentional 45-degree roof.
  ring ^= m.Manifold.cube([hi-lo+1,20,20]).translate([lo-.5,c[1]-1.7,c[2]-10])
  sides.append(max((ring^ss).volume()/ring.volume() for ss in frame_solids))
 frame_pins.append(dict(part=pin['id'],socket_wall_fraction=sides,pass_check=min(sides)>.95))
frame_joint=dict(frame_parts=[p['id'] for p in frame_parts],seam_x_mm=28.5,seam_gap_mm=.4,material_in_seam_mm3=seam_volume,pins=frame_pins,pass_check=len(frame_parts)==2 and seam_volume<.001 and all(x['pass_check'] for x in frame_pins))
report=dict(removable_frame_fixtures=fixturechecks,frame_joint=frame_joint,geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),removable_bearing_cheeks=checks,removable_transmission_walls=wallchecks,unassigned_fixed_components=unassigned,revision_connections=revision,attachment_pass=revision['connection_pass'] and frame_joint['pass_check'] and not unassigned and all(x['pass_check'] for x in checks+wallchecks+fixturechecks),limitations=['Checks attachment geometry, not pin-fit stiffness, layer adhesion or loaded deflection.','Bearing holes are perpendicular to the supplied wall print orientation; overhang and pin-hole finishing still need slicer review.'])
(O/'Bearing attachment checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
