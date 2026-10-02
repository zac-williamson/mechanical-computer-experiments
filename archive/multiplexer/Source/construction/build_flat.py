from geometry import *
from clean_print_mesh import clean
import contextlib,io
with contextlib.redirect_stdout(io.StringIO()):import build_local_support as L
OUT=Path(__file__).resolve().parent/'Flat-actuator';OUT.mkdir(exist_ok=True)
S=15.85;G=14.5;P={}
def flat(s):return s.translate([0,-L.SY,-L.G]).rotate([-90,0,0]).translate([0,S,G])
yi=L.yi
for side in [-1,1]:
 a,b=sorted([side*24.2,side*31.8]);w=box([a,20,-27.2],[b,28,20.8])
 for y,z in [(10.2,-18.4),(yi,-9.2),(10.2,0),(S,G)]:
  r=5.5 if z==G else 4.3
  w+=cyl(r,a,b,0,(0,y,z))+box([a,y,z-r],[b,24,z+r]);w-=cyl(2.65,a-.1,b+.1,0,(0,y,z))
 for y,z in [(24,-22.4),(24,12.5)]:w-=cyl(2.45,a-1,b+1,0,(0,y,z))
 aa,bb=sorted([side*24.1,side*26.2]);w-=box([aa,19.8,-26.6],[bb,28.2,-18.2])+box([aa,20,8.5],[bb,28,16.5])
 P[('Left' if side<0 else 'Right')+' side frame']=w
P['Right side frame']-=box([24,19.45,17.8],[33,29,22])
P['Right side frame']+=flat(extr(unary_union([L.capsule((24.2,0),pin,3.8) for pin in L.pins]),L.G-4.2,7.8)) ^ box([24.2,-100,-100],[100,100,100])
P['Front gear crossmember']=L.front
rail=box([-20.6,21.2,6.2],[20.6,27.2,10.6])
for side in [-1,1]:
 a,b=sorted([side*20.6,side*24]);rail+=box([a,20.2,6.2],[b,27.8,16.3])
 a,b=sorted([side*24,side*26]);rail+=box([a,20.2,8.7],[b,27.8,16.3])
 rail-=cyl(2.45,side*24-8.2,side*24+8.2,0,(0,24,12.5))
P['Keyed carriage rail']=rail
for n in ['Front bearing cheek','Rear bearing cheek','Short lever']:P[n]=flat(L.P[n])
c=box([-15.6,18.4,3.4],[15.6,28,13.4])
for a,b in [(-15.6,-8),(8,15.6)]:c+=cyl(5.5,a,b,0,(0,S,G))+box([a,S,11.5],[b,24,18])
fork=load('Left carriage half')^box([-8,0,-10],[8,20,7.5])
c+=fork+box([-2.5,11.4,5.5],[2.5,15,9])+box([-6,14,5.5],[6,21.2,9])
# Retain the stop profile itself, not the obsolete left pillar and diagonal brace.
# The new rear web carries the roof directly to the paired carriage joints.
c+=flat(extr(L.st,L.G-4,8.4)) ^ box([-7.8,-100,-100],[100,100,100])
# Remove the unused left stop-profile tail; retain the upper load-carrying cap.
c-=extr(Polygon([(-7.81,25),(-2.2,25),(-2.2,35.79),(-7.81,41.4)]),0,8.42).rotate([90,0,0]).translate([0,20.26,0])
# Bed-rooted fork neck: its running-ring clearance is cut below with the rest of c.
c+=box([-7.8,11.4,5.5],[6,21.2,9])
# Open-bottom guide: selector axle captures the carriage; rail sidewalls resist roll.
# Opening allows the one-piece rail to be lowered in before mounting the side frames.
c-=box([-30,20.8,5.8],[30,30,11])
for a,b in [(-19.1,-7.8),(7.8,15.7)]:c-=cyl(2.85,a,b,0,(0,S,G))
c-=cyl(3.45,-10,-7.8,0,(0,S,G))+cyl(3.45,7.8,10,0,(0,S,G))
c-=cyl(5.4,-7.9,7.9,0,(0,S,G))
for a,b in [(-30,-7.3),(7.3,30)]:c-=cyl(9.15,a,b,0,(0,10.2,0))
c-=flat(extr(L.capsule((-4.575,L.GY),(4.6,L.GY),6.1),L.G-4.5,9))
# Keep only the tested fork inside the driving-ring envelope; the short neck clears it.
c=(c-cyl(7.6,-11.8,11.8,0,(0,10.2,0)))+fork
# Blend the gear-clearance step into the fork carrier, rather than leave a ceiling.
c-=m.Manifold.cylinder(1.55,9.15,7.6,circular_segments=96).rotate([0,90,0]).translate([-7.3,10.2,0])
P['Single braced carriage']=c
# Relieve both band legs over the complete lever range, including their exit from each peg.
from shapely.geometry import LineString
from shapely import affinity
am=L.am;af=L.af
fixed_cut=[];lever_cut=[]
for beta in np.linspace(-28.5,28.5,115):
 r=np.radians(beta);R=np.array([[np.cos(r),-np.sin(r)],[np.sin(r),np.cos(r)]])
 bm=R@(am-L.pv)+L.pv
 line=LineString([af,bm]);ring=line.buffer(3.5,quad_segs=24)-line.buffer(1.7,quad_segs=24)
 fixed_cut.append(ring);lever_cut.append(affinity.rotate(ring,-float(beta),origin=tuple(L.pv)))
P['Right side frame']-=flat(extr(unary_union(fixed_cut),L.G-.9,1.8))
P['Short lever']-=flat(extr(unary_union(lever_cut),L.G-.9,1.8))
# Pin collars need pockets at the keyed joints, not friction-fit shaft holes.
for side in [-1,1]:
 for n,y,z in [('Front gear crossmember',24,-22.4),('Keyed carriage rail',24,12.5),('Left side frame' if side<0 else 'Right side frame',24,-22.4),('Left side frame' if side<0 else 'Right side frame',24,12.5)]:
  P[n]-=cyl(3.45,side*24-.95,side*24+.95,0,(0,y,z))
for x,y in L.pins:
 z=G+L.SY-y
 pocket=cyl(3.45,10.5,12.4,1,(x,0,z))
 P['Front bearing cheek']-=pocket;P['Right side frame']-=pocket
# Four independent bearing walls mount directly into a purpose-made base.
# Delete both crossmembers and all transverse frame joints.
for side in [-1,1]:
 name=('Left' if side<0 else 'Right')+' side frame'
 a,b=sorted([side*24.2,side*31.8])
 w=box([a,20,-22.7],[b,30.4,20.0])
 for y,z in [(10.2,-18.4),(yi,-9.2),(10.2,0),(S,G)]:
  r=5.5 if z==G else 4.3
  w+=cyl(r,a,b,0,(0,y,z))+box([a,y,z-r],[b,24,z+r])
  w-=cyl(2.65,a-.1,b+.1,0,(0,y,z))
 if side>0:
  w-=box([24,19.45,17.8],[33,29,22])
  w+=(flat(extr(unary_union([L.capsule((24.2,0),pin,3.8) for pin in L.pins]),L.G-4.2,7.8)) ^ box([24.2,-100,-100],[100,100,100]))
  w-=cyl(4.3,31.4,36.6,0,(0,S,G)) # rotating selector half-bush clearance
  w-=flat(extr(unary_union(fixed_cut),L.G-.9,1.8))
  for x,y in L.pins:w-=cyl(3.45,10.5,12.4,1,(x,0,G+L.SY-y))
 P[name]=w
 a,b=sorted([side*4.2,side*11.8])
 w=box([a,20,-22.7],[b,30.4,-4.9])
 for y,z in [(10.2,-18.4),(yi,-9.2)]:
  w+=cyl(4.3,a,b,0,(0,y,z))+box([a,y,z-4.3],[b,23,z+4.3])
  w-=cyl(2.65,a-.1,b+.1,0,(0,y,z))
  ca,cb=sorted([side*7.8,side*12]);w-=cyl(3.95,ca,cb,0,(0,y,z))
 P[('Left' if side<0 else 'Right')+' inner bearing wall']=w
P.pop('Front gear crossmember');P.pop('Keyed carriage rail')
board=box([-34,30.4,-26.7],[34,38.4,36])
# Windows follow the actual wall footprints rather than a generic hole grid.
for a,b in [(-20,-12.2),(-3.8,3.8),(12.2,20)]:
 board-=box([a,30.3,-20],[b,38.5,-5])
board-=box([-20,30.3,12],[20,38.5,28])
# The anti-roll guide belongs to the base: no second rail/frame connection layer.
board+=box([-23.8,21.2,6.2],[23.8,27.2,10.6])
for a,b in [(-23.8,-20.8),(20.8,23.8)]:
 board+=box([a,26.9,6.2],[b,30.6,10.6])
mounts=[]
for side in [-1,1]:
 for suffix,x,zs in [('side frame',side*28,[-18.4,10]),('inner bearing wall',side*8,[-18.4,-9.2])]:
  name=('Left' if side<0 else 'Right')+' '+suffix
  # A shallow rectangular tongue locates each wall and carries lateral load.
  z=sum(zs)/2;hz=3.2 if suffix=='side frame' else .8
  P[name]+=box([x-3.8,30.2,z-hz],[x+3.8,32.4,z+hz])
  board-=box([x-4,30.3,z-hz-.2],[x+4,32.6,z+hz+.2])
  for z in zs:
   P[name]-=cyl(2.45,22.3,32.5,1,(x,0,z))+cyl(3.45,29.45,31.35,1,(x,0,z))
   board-=cyl(2.45,30.3,38.5,1,(x,0,z))+cyl(3.45,30.3,31.35,1,(x,0,z))
   mounts.append(dict(wall=name,x=x,z=z))
# Support the anti-roll guide directly from the board; no suspended bridge underside.
board+=box([-20.8,27.2,6.2],[20.8,30.4,10.6])
# Horizontal friction-pin bores: retain the circular gripping sides, but use
# 45-degree roof flanks and a 1.3 mm closing bridge instead of a round ceiling.
for mount in mounts:
 name=mount['wall'];x,z=mount['x'],mount['z'];up=1 if x>0 else -1
 r=2.45;h=r+.4;w=r*2**.5-h
 poly=Polygon([(x+up*(r/2**.5-.02),z-r/2**.5),(x+up*h,z-w),(x+up*h,z+w),(x+up*(r/2**.5-.02),z+r/2**.5)])
 P[name]-=extr(poly,0,10.2).rotate([90,0,0]).translate([0,32.5,0])
 # The collar recess opens at the mounting face: no thin, supported collar roof.
 a,b=sorted([x,x+up*5])
 P[name]-=box([a,29.45,z-3.45],[b,32.5,z+3.45])
P['Common baseboard']=board
(OUT/'baseboard-mounts.json').write_text(json.dumps(mounts,indent=2))
# An open access scallop in the front cheek exposes the fixed band's free end.
ax,az=L.af[0],G+L.SY-L.af[1]
# Fit the band before installing this cheek; retain its complete pin-mount wall.
P['Right side frame']-=cyl(2.65,24,45,0,(0,S,G))
# Re-bore both cartridge pin paths after uniting the anchor web and spine.
for x,y in L.pins:
 P['Right side frame']-=cyl(2.45,11.3,20.4,1,(x,0,G+L.SY-y))
# Front cheek has one flat outer face: remove projecting pin bosses.
P['Front bearing cheek'] ^= box([-100,7.45,-100],[100,100,100])
# Moving anchor: leave its load-carrying web only behind the groove.
# The front lip is a free end, with room for a closed loop to pass over it.
mx,mz=L.am[0],G+L.SY-L.am[1]
P['Short lever']-=cyl(4.9,11.5,16.75,1,(mx,0,mz))-cyl(3.2,11.4,16.85,1,(mx,0,mz))
P['Short lever']-=cyl(4.9,14.95,16.75,1,(mx,0,mz))-cyl(2,14.85,16.85,1,(mx,0,mz))
# 45-degree underside on the free lip avoids a horizontal support shelf.
flare=m.Manifold.cylinder(1.2,3.2,2,circular_segments=96).rotate([-90,0,0]).translate([mx,13.65,mz])
P['Short lever']-=cyl(3.21,13.65,14.85,1,(mx,0,mz))-flare
# Preserve the actual 2 mm post root; the conservative band sweep must not notch it.
P['Short lever']+=cyl(2,13.65,20.0,1,(mx,0,mz))
# Separate worm-bearing supports: their inward thrust faces lie on the print bed.
car=P.pop('Single braced carriage')
# Smooth tapered bore mouths, without an abrupt inward-printing counterbore shelf.
for side in [-1,1]:
 a,b=sorted([side*8,side*10])
 flare=m.Manifold.cylinder(2,3.45 if side>0 else 2.85,2.85 if side>0 else 3.45,circular_segments=96).rotate([0,90,0]).translate([a,S,G])
 car+=cyl(3.5,a,b,0,(0,S,G))-flare
car+=box([-15.6,20.2,-1.8],[15.6,27.8,5.8])
car+=box([-15.6,25.8,14.2],[15.6,33.4,21.8])
car+=box([-15.6,25.8,12],[15.6,28,18])
# Rear central web keeps the roof/fork carrier one piece, behind the actuator.
# Match the stop roof outer Z edge so the lower backing does not leave a ledge.
roof_z_max=G+L.SY-L.st.bounds[1]
car+=box([-7.8,25.8,18],[7.8,29.8,roof_z_max])
car+=box([-7.8,19.85,41.4],[26.6,29.8,roof_z_max])
car+=box([-7.8,20.25,39.8],[26.6,29.8,roof_z_max])
join_positions=[(24.,2.),(29.6,18.)]
for side in [-1,1]:
 for y,z in join_positions:
  car-=cyl(2.45,-17,17,0,(0,y,z))
  car-=cyl(3.45,side*8.2-.95,side*8.2+.95,0,(0,y,z))
  # Chamfer both collar-pocket shoulders, rather than printing an annular ceiling.
  center=side*8.2
  for xa,ra,rb in [(center-1.95,2.45,3.45),(center+.95,3.45,2.45)]:
   car-=m.Manifold.cylinder(1.0,ra,rb,circular_segments=96).rotate([0,90,0]).translate([xa,y,z])
# Replace the steep reaction-clearance return with a 45-degree printable return.
for side in [-1,1]:
 poly=Polygon([(side*8,17.20),(side*30,39.20),(side*30,60),(side*8,60)])
 car-=(extr(poly,0,9).rotate([90,0,0]).translate([0,20.35,0]) ^ box([-100,-100,-100],[100,100,25.2]))
left=car^box([-100,-100,-100],[-8,100,25.2])
right=car^box([8,-100,-100],[100,100,25.2])
body=car-box([-100,-100,-100],[-7.8,100,25.4])-box([7.8,-100,-100],[100,100,25.4])
body+=fork # retain the physically tested clutch fork completely across the split
P['Left carriage bearing support']=left
P['Right carriage bearing support']=right
P['Carriage fork and roof']=body
# Pin ends enter the existing board window at the extremes of carriage travel.
P['Common baseboard']-=box([-22,30.3,12],[22,38.5,28])
mesh(left+right+body).export(OUT/'Single braced carriage.stl',file_type='stl_ascii') # aggregate for sweep checks only
P['Rear bearing cheek'] ^= box([-100,-100,-100],[100,24.25,100])
# Rear cheek and anchor web share one planar profile and a common bed face.
profile=unary_union([L.web,L.capsule(L.pins[0],L.af,3.4)])
rear=flat(extr(profile,L.G+4.4,4.0))
for x,y in L.pins:
 z=20.2-y
 rear+=cyl(4.15,19.65,20.30,1,(x,0,z))
 rear-=cyl(2.45,19.5,24.4,1,(x,0,z))
for x,y in [(0,L.GY),tuple(L.pv)]:rear-=cyl(2.65,20.1,24.4,1,(x,0,20.2-y))
rear-=cyl(5.4,-13,13,0,(0,S,G))
rear+=cyl(3.2,11.65,20.30,1,(ax,0,az))
rear-=cyl(3.3,14.95,16.75,1,(ax,0,az))-cyl(2,14.85,16.85,1,(ax,0,az))
flare=m.Manifold.cylinder(1.2,3.2,2,circular_segments=96).rotate([-90,0,0]).translate([ax,13.75,az])
rear-=cyl(3.3,13.75,14.95,1,(ax,0,az))-flare
P['Rear bearing cheek']=rear
# The separately printed anchor passes through a clearance opening in the frame.
P['Right side frame']-=cyl(3.4,11.3,19.7,1,(ax,0,az))
# Pointed roof lets this transverse clearance opening close at 45 degrees.
r=3.4;poly=Polygon([(ax+r/2**.5-.02,az-r/2**.5),(ax+r*2**.5,az),(ax+r/2**.5-.02,az+r/2**.5)])
P['Right side frame']-=extr(poly,0,8.4).rotate([90,0,0]).translate([0,19.7,0])
for x,y in L.pins:
 z=20.2-y;r=2.45;h=r+.4;w=r*2**.5-h
 poly=Polygon([(x+r/2**.5-.02,z-r/2**.5),(x+h,z-w),(x+h,z+w),(x+r/2**.5-.02,z+r/2**.5)])
 P['Right side frame']-=extr(poly,0,9.1).rotate([90,0,0]).translate([0,20.4,0])
 P['Right side frame']-=box([x,10.5,z-3.45],[x+5,12.4,z+3.45])

meta=[]
for n,s in P.items():
 s=s.simplify(0.005)
 t=mesh(s);t.export(OUT/(n+'.stl'),file_type='stl_ascii');pr=t.copy()
 axis=[0,1,0] if n in ['Single braced carriage','Left side frame','Right side frame','Front gear crossmember','Keyed carriage rail','Left inner bearing wall','Right inner bearing wall','Left carriage bearing support','Right carriage bearing support','Carriage fork and roof'] else [1,0,0]
 angle=-np.pi/2 if n in ['Single braced carriage','Right side frame','Common baseboard','Short lever','Right carriage bearing support','Carriage fork and roof','Rear bearing cheek','Right inner bearing wall'] else np.pi/2
 pr.apply_transform(trimesh.transformations.rotation_matrix(angle,axis));pr.apply_translation(-pr.bounds[0]);pr=clean(pr);assert pr.is_watertight,n;pr.export(OUT/(n+' - print.stl'),file_type='stl_ascii')
 for suffix in ['.stl',' - print.stl']:
  path=OUT/(n+suffix);verified=clean(trimesh.load(path));assert verified.is_watertight and len(verified.split())==1,n;verified.export(path,file_type='stl_ascii')
 meta.append(dict(id=n,print_rotation_axis=axis,print_rotation_angle=float(angle),bounds=t.bounds.tolist(),volume_mm3=float(t.volume),solids=[float(x.volume()) for x in s.decompose()],watertight=bool(t.is_watertight),motion='carriage' if n in ['Left carriage bearing support','Right carriage bearing support','Carriage fork and roof'] else 'rocker' if n=='Short lever' else 'fixed'))
(OUT/'printed-parts.json').write_text(json.dumps(meta,indent=2))
(OUT/'parameters.json').write_text(json.dumps(dict(worm_center=[0,S,G],reaction_center=[0,S,G+8],pivot=[L.pv[0],S,G+L.SY-L.pv[1]],rotation_axis=[0,1,0],status='experimental prototype; physical qualification pending'),indent=2))
print(json.dumps(meta,indent=2))
