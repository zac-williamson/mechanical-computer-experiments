import os
"""Fresh modular adaptation of the planar register memory mechanism.
No imports, meshes or construction operations from the rejected modular design.
"""
from pathlib import Path
import json,hashlib,base64,gzip,math,sys
import numpy as np,trimesh,manifold3d as m
ROOT=Path('/Users/zac/Documents/ChatGPT/lego designs 2_')
SRC=ROOT/'latest-register-analysis/planar-register/register-from-multiplexer/Planar register'
OUT=Path(os.environ.get('PLANAR_OUTPUT',str(ROOT/'work/planar-module-restart/adapted')));OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(ROOT/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
parts=[];provenance=[]
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=False)
def source(n):
 path=SRC/(n+'.stl');provenance.append(dict(part=n,source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()));return solid(trimesh.load(path))
def fold(s):return s.translate([0,-10.2,-16]).rotate([-90,0,0]).translate([0,10.2,16])
def add(n,s,motion='fixed',bed='rear',color=None):
 from clean_print_mesh import clean
 original_s=s;s=s.simplify(.001)
 topology_delta=abs((original_s-s).volume())+abs((s-original_s).volume())
 assert topology_delta<.1,(n,'initial topology cleanup changed geometry')
 original_s=s
 t=mesh(s);t.vertices=t.vertices.astype('<f4')
 t=trimesh.Trimesh(t.triangles.reshape(-1,3),np.arange(len(t.faces)*3).reshape(-1,3),process=True)
 repaired=solid(t).simplify(.001)
 delta=abs((original_s-repaired).volume())+abs((repaired-original_s).volume())
 print('Export cleanup',n,'geometric difference',delta,flush=True)
 if delta>=.1:
  print('Difference diagnostics',n,'original',original_s.volume(),'repaired',repaired.volume(),'removed',(original_s-repaired).bounding_box(),'added',(repaired-original_s).bounding_box(),flush=True)
  for suffix,ss in [('raw',original_s),('repaired',repaired),('removed',original_s-repaired),('added',repaired-original_s)]:
   mm=ss.to_mesh64();np.savez(OUT/(n+' '+suffix+'.npz'),vertices=mm.vert_properties[:,:3],faces=mm.tri_verts)
 assert delta<.1,(n,'export cleanup changed geometry')
 t=mesh(repaired);t.vertices=t.vertices.astype('<f4')
 t=trimesh.Trimesh(t.triangles.reshape(-1,3),np.arange(len(t.faces)*3).reshape(-1,3),process=True)
 t=clean(t)
 assert t.is_watertight,n
 rt=trimesh.Trimesh(t.triangles.reshape(-1,3),np.arange(len(t.faces)*3).reshape(-1,3),process=True)
 assert rt.is_watertight,(n,'STL round-trip topology')
 s=solid(t);assert s.status()==m.Error.NoError,n
 cs=[q for q in s.decompose() if q.volume()>1e-5]
 assert len(cs)==1,(n,'disconnected printable solids',len(cs))
 print(n,'solids',len(cs),'bounds',np.round(t.bounds,2).tolist(),flush=True)
 parts.append(dict(name=n,s=s,mesh=t,motion=motion,kind='printed',bed=bed,color=color or ([.12,.53,.62] if motion=='carriage' else [.83,.64,.2] if motion=='bolt' else [.18,.44,.45])))
# Rotate the original actuator working profiles rigidly about its input axle.
rawleft=source('Memory — Carriage fork and roof');rawright=source('Memory — Right carriage bearing support')
uppermask=box([-100,-100,9.5],[100,18,23.5])+box([-100,-100,23.5],[100,14.6,100])
upperleft=fold(rawleft^uppermask)^box([-100,-100,9.5],[100,100,100]);upperright=fold(rawright^uppermask)^box([-100,-100,9.5],[100,100,100])
lowleft=rawleft^box([-100,-100,-100],[100,100,9.5]);lowright=rawright^box([-100,-100,-100],[100,100,9.5])
lowleft-=box([-100,12.8,7.2],[100,100,10]);lowright-=box([-100,12.8,7.2],[100,100,10])
# Retain the existing two locking pockets, translating the entire keeper region.
keeper=(rawleft^box([-100,16.6,38.8],[4,23,100])).translate([0,6,-5.7])
front=fold(source('Memory — Front bearing cheek'))^box([-100,-100,-100],[36,100,100])
rawrear=source('Memory — Rear bearing cheek')
rear=fold(rawrear)^box([-100,-100,-100],[36,100,100])
# Keep the original band seat, shifted 0.6 mm toward -X to stay within the base.
rear+=fold((rawrear^box([33.7,5.9,24.6],[40.7,18.7,31.7])).translate([-.6,0,0]))
# The upper cheek carries the mounting tower. Both cheeks now grow directly
# from their respective Z bed faces, without the old tower's overhanging notch.
front+=box([30.6,27.6,20.4],[38.2,42.4,24.4])
rear+=box([30.6,27.6,7.6],[38.2,42.4,11.6])
for y in [31.4,39.4]:
 front+=cy(4.75,20.4,28.4,2,[35.25,y,0])
 rear+=cy(4.75,7.6,19.6,2,[35.25,y,0])^box([30.8,-100,-100],[40,100,100])
front+=box([29.5,32.4,20.4],[40,39.6,43.9])
for y in [31.4,39.4]:
 front-=cy(2.5,20.3,28.5,2,[35.25,y,0])
 rear-=cy(2.5,7.5,19.7,2,[35.25,y,0])
for z in [31.2,39.2]:front-=cy(2.5,31.9,39.7,1,[34.75,0,z])
rear-=front
lever=fold(source('Memory — Short lever'))
# Swept spaces for fixed actuator cheeks. Preserve source working profiles;
# these cuts apply only to the newly added structural webs.
def sweep(s,steps=9):return sum((s.translate([float(q),0,0]) for q in np.linspace(-3.85,3.85,steps)),m.Manifold())
clear=sweep(front+rear)
# Conservative rotating shaft and retainer envelopes (not geared contact).
for x,y in [(0,18.2),(13.192323604,26.328448698)]:
 for r,za,zb in [(3.8,24.3,28.9),(2.7,28.6,30.1),(3.6,5.6,7.6)]:
  clear+=cy(r,za,zb,2,[x-3.85,y,0])+cy(r,za,zb,2,[x+3.85,y,0])+box([x-3.85,y-r,za],[x+3.85,y+r,zb])
left=lowleft+upperleft+keeper;right=lowright+upperright
# Broad continuous side webs; no separate rear carriage piece.
for isright,a,b in [(False,-15.6,-8),(True,8,15.6)]:
 web=box([a,2,9],[b,6,38.8])+box([a,2,35.8],[b,22.4,38.8])
 web+=box([a,14.4,28.2],[b,38.4,36])
 web-=clear
 x=11.0 if isright else -11.0
 web-=cy(2.5,14.3,22.5,1,[x,0,32])+cy(3.25,14.3,14.8,1,[x,0,32])
 if isright:right+=web
 else:left+=web
left+=box([-15.6,30.4,28],[7.8,38.4,36])+box([-15.6,22.8,35.5],[-8,39.5,39])
# The added webs do not fill the source's two locking pockets.
keeper_void=box([-12,22.8,33.1],[2,28.8,39.61])-keeper
left-=keeper_void
# Rear posts join the original lower fork to the rotated upper mechanism.
# The right post is behind the original roof, with a 0.3 mm separation.
left+=box([-15.6,35.2,2],[-9.5,43.8,36])+box([-15.6,24,2],[-9.5,43.8,6])
right+=box([8,39.8,2],[15.6,43.8,36])+box([8,24,2],[15.6,43.8,6])+box([8,30.4,28],[15.6,43.8,36])
# Enclose the front joining bore below the axle-head clearance.
left+=box([-15.6,20,-3.5],[7.8,28,7.2])
right+=box([8,20,-3.5],[15.6,28,7.2])
for name in ['left','right']:
 s=locals()[name]
 for y,z in [(24,.5),(34.4,32)]:s-=cy(2.5,.1,16.3,0,[0,y,z])+cy(3.3,7.8,8.6,0,[0,y,z])
 s-=cy(2.65,-20,20,0,[0,10.2,16])
 for cx,yy in [(0,18.2),(13.192323604,26.328448698)]:
  head=cy(3.4,5.8,7.8,2,[cx-3.85,yy,0])+cy(3.4,5.8,7.8,2,[cx+3.85,yy,0])+box([cx-3.85,yy-3.4,5.8],[cx+3.85,yy+3.4,7.8])
  head-=cy(7.6,-100,100,0,[0,10.2,0])
  s-=head
 locals()[name]=s
exec(compile((ROOT/'work/planar-module-restart/carriage-strength/reinforce.py').read_text(),'reinforce.py','exec'))
add('Carriage body',left,'carriage','left');add('Carriage bearing end',right,'carriage','right')
add('Upper actuator cheek',front,bed='bottom',color=[.83,.65,.25]);add('Lower actuator cheek',rear,bed='bottom',color=[.83,.65,.25]);add('Actuator lever',lever,'rocker','bottom',color=[.83,.65,.25])
# Original bolt and guide: retain the lower head, tip and retaining surfaces.
# Relocate them +6Y and -6.3Z, then shorten only the unused upper head.
originalbolt=source('Direct lock bolt');shift=[0,6,-5.7]
# Retain the source head, axle bore and tip; remove its tall rear band arm.
bolt=originalbolt.translate(shift)^box([-100,-100,-100],[100,31.2,52.2])
guide=source('Detachable bolt guide').translate(shift)^box([-100,-100,-100],[100,100,55.8])
# Paired short side seats keep band force close to the guided head. The
# source's front/rear retaining faces and tip guide remain in place.
for sign in [-1,1]:
 edge=-5.05+sign*6
 lo,hi=sorted([edge-sign*.2,edge+sign*3])
 hook=cy(1.4,lo,hi,0,[0,27.6,49.6])
 lo,hi=sorted([edge+sign*2.75,edge+sign*3.55])
 hook+=cy(2,lo,hi,0,[0,27.6,49.6]);bolt+=hook
 lo,hi=sorted([edge-sign*.1,edge+sign*4])
 guide-=box([lo,25.2,47.2],[hi,30,56])
 band_x=-5.05+sign*8.05
 guide-=box([band_x-.8,25.2,40.4],[band_x+.8,30,56])
 lo,hi=sorted([edge+sign*.4,edge+sign*3])
 anchor=cy(1.4,lo,hi,0,[0,27.6,42.8])
 lo,hi=sorted([edge+sign*2.75,edge+sign*3.55])
 anchor+=cy(2,lo,hi,0,[0,27.6,42.8]);guide+=anchor
# The old rear band anchor is not used.
guide-=box([-9,37.3,40.2],[-1.1,42,47.2])
# Flat-bottomed guide with a rear backing plate and two Y-axis frame pins.
# Moving bolt surfaces remain in front of Y31.6; the follower has an open slot.
guide=guide^box([-19.9,-100,39.8],[19.9,38,100])
guide+=box([-19.9,32.6,39.8],[19.9,38,55.8])
guide-=box([-9.05,31.5,43.6],[-1.05,38.1,56])
def peaked_pin_y(x,z,a,b):
 r=2.5;d=r/(2**.5)
 roof=m.CrossSection([[(x-d,z+d),(x+d,z+d),(x,z+r*2**.5)]]).extrude(b-a).rotate([90,0,0]).translate([0,b,0])
 return cy(r,a,b,1,[x,0,z])+roof
for x in [-14,14]:
 guide-=peaked_pin_y(x,45.05,30.1,38.1)+cy(3.25,37.8,38.1,1,[x,0,45.05])
# Thicker roots, retaining the original band grooves and free hook ends.
for sign in [-1,1]:
 edge=-5.05+sign*6
 a,b=sorted([edge-sign*.4,edge+sign*.4])
 bolt+=cy(2.2,a,b,0,[0,27.6,49.6])
 a,b=sorted([edge+sign*.4,edge+sign*1.2])
 guide+=cy(2.2,a,b,0,[0,27.6,42.8])
add('Locking bolt',bolt,'bolt','front',color=[.88,.65,.16]);add('Locking bolt guide',guide,bed='bottom')
# Rod ends retain 95.6 mm overall length and two holes on an 8 mm pitch.
def rod(z,h):
 s=box([-47.8,6.2,z-h/2],[47.8,14.2,z+h/2])
 s-=box([-48,6.1,z-h],[-32,10.6,z+h])+box([32,9.8,z-h],[48,14.3,z+h])
 for x in [-44,-36,36,44]:s-=cy(2.5,6.1,14.3,1,[x,0,z])
 return s
move=rod(32,7.6)
for x in [-11.0,11.0]:move-=cy(2.5,6.1,14.3,1,[x,0,32])
add('Carriage control rod',move,'carriage','rear',color=[.12,.53,.62])
rad=12.;lo=0.;hi=rad-3.75
for _ in range(60):
 cx=(lo+hi)/2;rise=math.sqrt(rad*rad-(3.75-cx)**2)-math.sqrt(rad*rad-(-3.75-cx)**2)
 if rise<3.8:lo=cx
 else:hi=cx
cx=(lo+hi)/2;cz=47.6+math.sqrt(rad*rad-(3.75-cx)**2)
lock=rod(48,12)-cy(rad+3.6,6.1,14.3,1,[-5.05+cx,0,cz])
# Integral shoulders face the inner bearing walls; the rod body passes through.
for a,b in [(-16.65,-13.65),(13.65,16.65)]:lock+=box([a,2.2,42],[b,6.3,46.5])
lock+=m.CrossSection([[[-13,42.1],[-11,40],[11,40],[13,42.1]]]).extrude(8).rotate([90,0,0]).translate([0,14.2,0])
add('Lock control rod',lock,'lock','rear',color=[.84,.4,.17])
# New modular frame: explicit 4 mm plate and solid sockets connected to it.
frame=box([-40,44.4,-8],[40,48.4,56])
for x in [-24.4,24.4]:
 frame+=box([x-4,32,-7],[x+4,48.4,56])
 for z in [0,50]:frame-=cy(2.5,31.9,40.1,1,[x,0,z])+cy(3.25,31.9,32.4,1,[x,0,z])
frame+=box([29.5,40.4,26.75],[40,48.4,43.9])
for z in [31.2,39.2]:frame-=cy(2.5,40.3,48.5,1,[34.75,0,z])
for x in [-14,14]:
 frame+=box([x-5.25,38.4,39.8],[x+5.25,48.4,50.6])
 frame-=cy(2.5,38.3,46.4,1,[x,0,45.05])+cy(3.25,38.3,38.6,1,[x,0,45.05])
for x in [-32,32]:
 for z in [0,48]:
  frame+=box([x-4,32,z-4],[x+4,48.4,z+4]);frame-=cy(2.5,31.9,40.1,1,[x,0,z])+cy(3.25,31.9,32.4,1,[x,0,z])
# Four-pad rod guides: 0.35 mm side clearance, 0.65 mm corner relief.
def hole(z,h,a,b):
 u=4.65;v=h/2+.65;w=1.2;d=.3
 poly=[[-u,-v],[-w,-v],[-w,-v+d],[w,-v+d],[w,-v],[u,-v],[u,-w],[u-d,-w],[u-d,w],[u,w],[u,v],[w,v],[w,v-d],[-w,v-d],[-w,v],[-u,v],[-u,w],[-u+d,w],[-u+d,-w],[-u,-w]]
 return m.CrossSection([poly]).extrude(b-a+.2).rotate([90,0,90]).translate([a-.1,10.2,z])
for sign in [-1,1]:
 a,b=(-24.4,-20.4) if sign<0 else (20.4,24.4);fa,fb=(-28.4,-20.4) if sign<0 else (20.4,28.4)
 wall=box([a,6.2,-7],[b,32,56])+box([a,2.2,7],[b,18.2,56])+box([fa,24,-7],[fb,32,56])+cy(7,a,b,0,[0,10.2,0])
 wall-=box([fa-.1,14.2,7.2],[fb+.1,33,25.4])
 for z in [0,16]:wall-=cy(2.65,a-.1,b+.1,0,[0,10.2,z])
 for z,h in [(32,7.6),(48,12)]:wall-=hole(z,h,a,b)
 for z in [0,50]:wall-=cy(2.5,23.9,32.1,1,[(fa+fb)/2,0,z])+cy(3.25,31.6,32.1,1,[(fa+fb)/2,0,z])
 add(('Left' if sign<0 else 'Right')+' bearing wall',wall,bed='right' if sign<0 else 'left')
frame-=box([-31.1,31.9,7.2],[31.1,44.3,25.4])
frame-=rear+front+guide
# Clear the upper end of the cheek joining pin below the mounting socket.
frame-=cy(2.8,26.65,28.5,2,[35.25,39.4,0])
add('Module base',frame)
# Geometry checkpoint. Preserve failures as diagnostics; do not label printable.
for p in parts:p['mesh'].export(OUT/(p['name']+'.stl'))
checks=[]
for q,l in [(-3.75,0),(-3.75,3.8),(0,3.8),(3.75,3.8),(3.75,0)]:
 pp=[]
 for p in parts:
  shift=[q if p['motion']=='carriage' else 3.75 if p['motion']=='lock' and l else -3.75 if p['motion']=='lock' else 0,0,l if p['motion']=='bolt' else 0]
  s=p['s'].translate(shift);pp.append((p,s,np.array(s.bounding_box()).reshape(2,3)))
 for i,(p,a,ab) in enumerate(pp):
  for r,b,bb in pp[i+1:]:
   if np.any(ab[1]<=bb[0]) or np.any(bb[1]<=ab[0]):continue
   vv=(a^b).volume()
   if vv>.02:checks.append(dict(q=q,lift=l,a=p['name'],b=r['name'],mm3=vv,bounds=(a^b).bounding_box()))
print('INTERFERENCES',json.dumps(checks,indent=2),flush=True)
(OUT/'Development checks.json').write_text(json.dumps(dict(provenance=provenance,interferences=checks),indent=2))
# Basic review mesh includes the source native hardware, rigidly rotated where necessary.
hw=json.loads((SRC/'hardware.json').read_text());hv=np.load(SRC/'hardware.npz')['vertices']
foldM=trimesh.transformations.rotation_matrix(-math.pi/2,[1,0,0],[0,10.2,16])
for p in hw:
 n=p['id'];local=n.removeprefix('Memory — ')
 if not n.startswith('Memory — '):continue
 if local in ['O-left 5L axle','O-right 5L axle','selector-right-retainer']:continue
 if local not in ['U015','U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer','C-shaft','selector-right-retainer','L072','L102','L097','L099','L069','L105','O-left 5L axle','O-right 5L axle']:continue
 a=hv[p['offset']//3:p['offset']//3+p['vertices']].copy()
 if local in ['U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer']:a=trimesh.transform_points(a,foldM)
 if local in ['reaction-stop-axle','pivot-stop-axle']:a[:,2]-=.6
 if local in ['L069','L105']:a[:,0]+=(26.6 if local=='L069' else -26.6)-a[:,0].mean()
 motion='worm' if local=='U015' else 'clutch-ring' if local=='L099' else 'fixed'
 parts.append(dict(name=local,mesh=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False),motion=motion,kind='native',color=[.45,.37,.2] if local in ['U015','U022'] else [.22,.25,.28]))
exec(compile((ROOT/'work/planar-module-restart/hardware.py').read_text(),str(ROOT/'work/planar-module-restart/hardware.py'),'exec'))
meta=[];arrays=[]
for p in parts:
 a=p['mesh'].triangles.reshape(-1,3);meta.append(dict(name=p['name'],motion=p['motion'],kind=p['kind'],color=p['color'],axis=p.get('axis'),mates=p.get('mates',[]),bed=p.get('bed'),offset=sum(x.size for x in arrays),vertices=len(a)));arrays.append(a)
D=dict(parts=meta,centre=[0,22,24],geometry=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode())
html=(ROOT/'work/planar-module-restart/viewer.html').read_text().replace('__DATA__',json.dumps(D,separators=(',',':'))).replace('Source-core inspection. Module connections and frame adaptation are in progress; this is not a print release.','Work in progress: hardware, movement and print checks are not complete.').replace('Starting from the planar register’s original memory mechanism. Two carriage parts; original actuator, clutch, bolt and guide.','Planar register adaptation: original working profiles, four adjacent rows, two carriage parts.').replace('?5.4:0','?3.8:0')
(OUT/'Viewer.html').write_text(html);(OUT/'Model.json').write_text(json.dumps(D,separators=(',',':')))
print('ADAPTED PREVIEW SAVED',flush=True)
