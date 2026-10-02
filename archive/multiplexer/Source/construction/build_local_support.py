from geometry import *
from shapely.geometry import box as polybox
from shapely import affinity
OUT=Path(__file__).resolve().parent/'Local-support';OUT.mkdir(exist_ok=True)
G=13.2;DY=-4.5;SY=10.2+DY;GY=2.2+DY;ANG=4.
oldpv=pv.copy();r=np.radians(ANG);R=np.array([[np.cos(r),-np.sin(r)],[np.sin(r),np.cos(r)]])
pv=R@(oldpv-[0,2.2])+[0,GY];delta=pv-oldpv
ARM_L=6.5;STOP_DX=.3125;STOP_DY=0.
pins=[(40,.7),(40,-12.5)];P={}
def capsule(a,b,r):return unary_union([Point(*a).buffer(r,quad_segs=32),Point(*b).buffer(r,quad_segs=32)]).convex_hull
# Two data paths remain LEGO 16 -> 8 -> 16, with independent opposing input ports.
yi=10.2+np.sqrt(144-9.2**2)
for side in [-1,1]:
 a,b=sorted([side*24.2,side*31.8]);wall=box([a,20,-27.2],[b,28,26.2])
 for y,z in [(10.2,-18.4),(yi,-9.2),(10.2,0),(SY,G)]:
  rad=5.5 if z==G else 4.3
  wall+=cyl(rad,a,b,0,(0,y,z))+box([a,y,z-rad],[b,24,z+rad]);wall-=cyl(2.65,a-.1,b+.1,0,(0,y,z))
 # Front and rear joints are positively keyed rectangular seats, not pin hinges.
 wall+=box([a,18.2,18.6],[b,25.8,26.2])
 for y,z in [(24,-22.4),(22,22.4)]:wall-=cyl(2.45,a-1,b+1,0,(0,y,z))
 # Rectangular crossmember tongues 2 mm deep; .2 mm clearance per side.
 if side<0:wall-=box([-26.2,19.8,-26.6],[-24.1,28.2,-18.2])+box([-26.2,18.0,18.4],[-24.1,26.0,26.4])
 else:wall-=box([24.1,19.8,-26.6],[26.2,28.2,-18.2])+box([24.1,18.0,18.4],[26.2,26.0,26.4])
 P[('Left' if side<0 else 'Right')+' side frame']=wall
# Front gear crossmember, with recessed bushes and both inner data/idler bearings.
front=box([-26,20,-26.4],[26,28,-18.4])
for side in [-1,1]:
 a,b=sorted([side*4.2,side*11.8]);s=box([a,20,-22.7],[b,28,-4.9])
 for y,z in [(10.2,-18.4),(yi,-9.2)]:
  s+=cyl(4.3,a,b,0,(0,y,z))+box([a,y,z-4.3],[b,23,z+4.3]);s-=cyl(2.65,a-.1,b+.1,0,(0,y,z))
  ca,cb=sorted([side*7.8,side*12]);s-=cyl(3.95,ca,cb,0,(0,y,z))
 front+=s;front-=cyl(2.45,side*24-8.2,side*24+8.2,0,(0,24,-22.4))
P['Front gear crossmember']=front
# Short, captured printed anti-roll rail. No stabilising axles.
rail=box([-20.6,20,10.8],[20.6,24,16.4])
# Rear mounts stay behind carriage sweep; webs go down and rearward at the ends.
for side in [-1,1]:
 a,b=sorted([side*20.6,side*24]);rail+=box([a,18.2,10.8],[b,25.8,26.2])
 a,b=sorted([side*24,side*26]);rail+=box([a,18.2,18.6],[b,25.8,26.2])
 rail-=cyl(2.45,side*24-8.2,side*24+8.2,0,(0,22,22.4))
P['Keyed carriage rail']=rail
# Integral local cartridge spine on right frame, confined to the upper right corner.
spine=extr(unary_union([capsule(pins[0],pins[1],3.8),capsule((30,0),pins[0],3.8)]),G-4.2,7.8)
spine+=box([28.2,-6.4,G-4.2],[31.8,6.5,G+3.6])
spine-=cyl(2.65,24,32.1,0,(0,SY,G))
for x,y in pins:spine-=cyl(2.45,G-5,G+5,2,(x,y,0))
P['Right side frame']+=spine
# Local bearing cheeks: two 4 mm bearing plates and two 3L friction pins total.
web=unary_union([capsule((0,GY),pv,4.2),capsule(pv,pins[0],4.2),capsule(pins[0],pins[1],4.2),Point(0,GY).buffer(5.1,quad_segs=48),Point(*pv).buffer(5.5,quad_segs=48)])
for name,z0,z1,b0,b1 in [('Front bearing cheek',G-8.4,G-4.4,G-12.4,G-4.4),('Rear bearing cheek',G+4.4,G+8.4,G+3.8,G+11.6)]:
 s=extr(web,z0,z1-z0)
 for x,y in pins:
  s+=cyl(4.2,b0,b1,2,(x,y,0));s-=cyl(2.45,b0-.2,b1+.2,2,(x,y,0))
  if name.startswith('Front'):s-=cyl(3.25,G-4.8,G-4.3,2,(x,y,0))
 for x,y in [(0,GY),tuple(pv)]:s-=cyl(2.65,z0-.1,z1+.1,2,(x,y,0))
 # Full spinning worm and travel allowance, plus .4 mm radial margin.
 s-=cyl(5.4,-13,13,0,(0,SY,G))
 P[name]=s
# Retained source tooth mesh is rotated rigidly with the reaction gear, not scaled.
teeth=load('Direct lever and band cleat')^box([-10,-8,28],[9,5,35.8])
teeth=teeth.translate([0,-2.2,0]).rotate([0,0,ANG]).translate([0,GY,G-32])
outline=Point(*pv).buffer(6.5,quad_segs=48)
for off in [(-ARM_L,-4),(8,-4),(9,1.5)]:
 rr=3.2 if off==(9,1.5) else 2.6
 outline=outline.union(unary_union([Point(*(pv+off)).buffer(rr,quad_segs=48),Point(*pv)]).convex_hull)
trim=np.load(OUT/f'trim-{int(ANG)}.npz');outline=Polygon(trim['world_profile'])
lever=extr(outline,G-4.2,8.4);lever-=cyl(2.65,G-5,G+5,2,(*pv,0))
# Band captured between printable flanges in the gear plane, without crossing a cheek.
am=pv+[9,1.5];af=pv+[24,4]
def grooved_post(body,xy):
 outer=cyl(3.4,G-1.0,G+1.0,2,(*xy,0));inner=cyl(2.,G-1.1,G+1.1,2,(*xy,0))
 return body-(outer-inner)
lever=grooved_post(lever,am);P['Short lever']=lever
anchor=cyl(3.2,G-4.2,G+3.6,2,(*af,0))+extr(capsule(pins[0],af,3.2),G+.8,2.8)
anchor=grooved_post(anchor,af);P['Right side frame']+=anchor
# Full carriage, separated into two interlocking parts with paired Z-axis pins.
body=box([-15.6,17.2,8.0],[15.6,26.8,19.2])-box([-16,19.6,10.4],[16,24.4,16.8])
for a,b in [(-15.6,-8),(8,15.6)]:
 body+=cyl(5.5,a,b,0,(0,SY,G))+box([a,SY,G-3],[b,19,G+5.8])
 body-=cyl(2.85,a-.1,b+.1,0,(0,SY,G))
body-=cyl(3.45,-10,-7.8,0,(0,SY,G))+cyl(3.45,7.8,10,0,(0,SY,G))
# Gear clearance scallops apply to the full allowed carriage travel.
for a,b in [(-30,-7.3),(7.3,30)]:body-=cyl(9.15,a,b,0,(0,10.2,0))
left=body^box([-100,-100,-100],[7.8,100,100]);right=body^box([8,-100,-100],[100,100,100])
# Short fork and lap: the large longitudinal fork block is eliminated.
fork=load('Left carriage half')^box([-8,0,-10],[8,20,7.5])
left+=fork+box([-2.5,11.4,6.5],[2.5,16.4,12.8])
lapbox=box([-7.2,11.4,5.2],[8,19,21.2]);left-=lapbox
left+=box([-7.2,11.4,5.2],[7.2,19,13.2])+fork
right+=box([-7.2,11.4,13.4],[15.6,19,21.2])
# Link the left lap to the left bearing and rail without encroaching on the worm.
left+=box([-15.6,11.4,8],[7.2,17.2,12.8])
# The original rounded stop lobes are translated; left lobe follows its shortened arm.
st=section('Left carriage half',40).intersection(polybox(-16,-20,27,-9))
st=affinity.translate(st,-oldpv[0],-oldpv[1]);sl=st.intersection(polybox(-50,-50,0,0));sr=st.intersection(polybox(0,-50,50,0))
st=unary_union([affinity.translate(sl,8-ARM_L+STOP_DX,STOP_DY),sr]);st=affinity.translate(st,*pv)
left+=extr(st,G-4,8)
left+=box([-15.6,-20+delta[1],G-4],[-10.4,SY+1,G+4])
left+=box([-15.6,-20+delta[1],G-4],[pv[0]-5,-16+delta[1],G+4])
# Original working bore details re-established through the pillar.
left-=cyl(2.85,-16,-7.8,0,(0,SY,G));left-=cyl(3.45,-10,-7.8,0,(0,SY,G))
for x in [-3.5,3.5]:
 hole=cyl(2.45,5.0,21.4,2,(x,15.2,0))+cyl(3.25,12.8,13.6,2,(x,15.2,0))
 left-=hole;right-=hole
# Conservative worm rotation envelope, excluding its intentional thrust end faces.
left-=cyl(5.4,-7.9,7.9,0,(0,SY,G));right-=cyl(5.4,-7.9,7.9,0,(0,SY,G))
for a,b in [(-30,-7.3),(7.3,30)]:
 left-=cyl(9.15,a,b,0,(0,10.2,0));right-=cyl(9.15,a,b,0,(0,10.2,0))
gear_track=extr(capsule((-4.575,GY),(4.6,GY),6.1),G-4.5,9.)
left-=gear_track;right-=gear_track
P['Left short carriage']=left;P['Right short carriage']=right
meta=[]
for name,s in P.items():
 t=mesh(s);t.export(OUT/(name+'.stl'));pr=t.copy()
 if 'carriage' in name or 'side frame' in name or 'crossmember' in name or 'rail' in name:pr.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))
 pr.apply_translation(-pr.bounds[0]);pr.export(OUT/(name+' - print.stl'))
 meta.append(dict(id=name,bounds=t.bounds.tolist(),volume_mm3=float(t.volume),solids=len(s.decompose()),watertight=bool(t.is_watertight),motion='carriage' if 'carriage' in name and 'rail' not in name else 'rocker' if name=='Short lever' else 'fixed'))
(OUT/'printed-parts.json').write_text(json.dumps(meta,indent=2))
params=dict(pivot=pv.tolist(),worm_center=[0,SY,G],reaction_center=[0,GY,G],actuator_rotation_degrees=ANG,band_moving=am.tolist()+[G],band_fixed=af.tolist()+[G],left_pad_arm=ARM_L,stop_dx=STOP_DX,stop_dy=STOP_DY,pins=pins)
(OUT/'parameters.json').write_text(json.dumps(params,indent=2));print(json.dumps(meta,indent=2))
