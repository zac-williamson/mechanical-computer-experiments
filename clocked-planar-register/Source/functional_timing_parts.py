"""Manufacturing split of timing assemblies; actual cam and fork-slot faces retained.

Cam and fork plates print from their common rear Y29.8 face in -Y.
Rear frames print from their common front Y30.2 face in +Y.
Pin joints are perpendicular to these bed faces; all running faces remain
bed-connected surfaces. The elastic-anchor retaining lip grows at 45 degrees.
"""
import numpy as np
from wall_flat_frame import project_back

def split_timing(g):
 P,A,PRINT=g['P'],g['A'],g['PRINT'];m=g['m'];box,cyl,sol,emit,native=[g[k] for k in ['box','cyl','sol','emit','native']]
 def toward_front(s,y):return project_back(s.scale([1,-1,1]),-y).scale([1,-1,1])
 def bore(x,z,lo,hi):return cyl(2.5,lo,hi,1,[x,0,z])
 def socket(x,z,lo,hi,face):
  # 45-degree collar entry; a bed-side square counterbore would form a roof.
  low=abs(face-lo)<abs(face-hi)
  if low:
   lip=cyl(3.3,face-.7,face+.7,1,[x,0,z]);y=face+.7;r0,r1=3.3,2.5
  else:
   lip=cyl(3.3,face-.7,face+.7,1,[x,0,z]);y=face-1.5;r0,r1=2.5,3.3
  taper=m.Manifold.cylinder(.8,r0,r1,circular_segments=32).rotate([-90,0,0]).translate([x,y,z])
  return bore(x,z,lo,hi)+lip+taper
 for bank in ['master','slave']:
  n=bank+' local timing bar';i=next(i for i,p in enumerate(P) if p['id']==n);old=sol(A[i]);P.pop(i);A.pop(i);PRINT.pop(n);g['S'].pop(n,None)
  gx=-60 if bank=='master' else 92;terminal=60 if bank=='master' else 70
  rear=toward_front(old^box([-300,30.2,-100],[300,80,150]),30.2)
  # Two remote pads support the other end of the separate cam plate.
  pads=[-74,-62] if bank=='master' else [198,210]
  cam=old^box([-300,-100,40],[300,29.8,150])
  # Extend only beyond the follower's entire working interval.
  if bank=='master':cam+=box([28.8,25.6,47],[64.8,29.8,49.7])
  else:
   cam+=box([186.8,25.6,47],[214.8,29.8,49.7])
   rear+=box([186.8,30.2,46.6],[214.8,32.2,54])
  cam+=box([terminal-4.8,22.2,46.6],[terminal+4.8,29.8,76.8])
  rear+=box([terminal-4.8,30.2,46.6],[terminal+4.8,37.8,76.8])
  for xx in pads:
   cam+=box([xx-4.8,22.2,45.2],[xx+4.8,29.8,54.8])
   rear+=box([xx-4.8,30.2,45.2],[xx+4.8,37.8,54.8])
   cam-=socket(xx,50,22.1,29.9,29.8);rear-=socket(xx,50,30.1,37.9,30.2)
   native(bank+' cam plate pin '+str(xx),'2780',[xx,30,50],bank,axis=1);P[-1]['motion']='crosshead'
  for zz in [60,72]:
   cam-=socket(terminal,zz,22.1,29.9,22.2);rear-=bore(terminal,zz,30.1,37.9)
  # The unchanged floating-fork slots are cut through both plates. Their
  # withdrawal shoulders and lost-motion distance are not replaced by rigid forks.
  front=old^box([gx-25,-100,-100],[gx+25,29.8,35])
  anchor=gx-12+6.5*(1 if bank=='master' else -1);z=7.8
  # Rebuild this small anchor analytically before projection. Projecting the
  # old triangulated circular lip produces coincident, zero-width faces.
  front-=box([anchor-3,16,4.7],[anchor+3,28.1,10.9])
  front=project_back(front,29.8)
  front+=box([anchor-2.5,20.2,6.6],[anchor+2.5,29.8,9.0])
  # Join outside the floating fork, below the stationary cheek carrier.
  # Only the existing rear shoe plane bridges between the two end pads.
  front+=box([gx-27.2,28.2,20],[gx+19.8,29.8,24.2])
  bias=1 if bank=='master' else -1
  for xx in [gx-10,gx+12]:
   # Reopen the inherited slot through the projected backing.
   def slot(r):return (cyl(r,28.1,38,1,[xx,0,13.2])+cyl(r,28.1,38,1,[xx-4*bias,0,13.2])).hull()
   front-=slot(2.7);front-=slot(3.4)^box([-300,28.1,-100],[300,29,150])
  # Both fixed pins sit on the clear input side. The inherited rear slot
  # cheeks still carry the withdrawal load directly; the front plate closes
  # their slots and carries the small elastic anchor, not the cam load.
  xx=gx-22
  front+=box([xx-5.2,22.2,14.0],[xx+5.2,29.8,36.3])
  rear+=box([xx-5.2,30.2,14.0],[xx+5.2,37.8,36.3])
  for zz in [19.1,31.1]:
   front-=socket(xx,zz,22.1,29.9,29.8);rear-=socket(xx,zz,30.1,37.9,30.2)
   native(bank+' fork plate pin '+str(zz),'2780',[xx,30,zz],bank,axis=1);P[-1]['motion']='crosshead'
  # Preserve the band's original plane and peg centre. Move only the head
  # forward and ramp its rear face; do not fill in the functional band groove.
  anchor=gx-12+6.5*bias;z=7.8
  front-=cyl(2.7,16.8,20.3,1,[anchor,0,z])
  front+=cyl(1.2,18.9,20.4,1,[anchor,0,z])
  lip=m.Manifold.cylinder(1.3,2.5,1.2,circular_segments=32).rotate([-90,0,0]).translate([anchor,17.6,z])
  front+=lip+cyl(2.5,16.9,17.6,1,[anchor,0,z])
  emit(bank+' flat cam plate',cam,bank,(1,-1),motion='crosshead')
  emit(bank+' fork drive plate',front,bank,(1,-1),motion='crosshead')
  emit(bank+' timing rear frame',rear,bank,(1,1),motion='crosshead')
 # The joining strap sits in front of the terminal pads. One 3L pin spans
 # strap / cam plate / rear frame, with 0.4 mm assembly gaps between layers.
 # Each latch retains its two terminal pins in standalone use; the joining
 # strap presses onto their exposed short ends without replacing internals.
 oldnames=[p['id'] for p in P if p['id'].startswith('Timing connection pin ')]
 for n in oldnames:
  i=next(i for i,p in enumerate(P) if p['id']==n);P.pop(i);A.pop(i)
 n='Removable timing connection';i=next(i for i,p in enumerate(P) if p['id']==n);P.pop(i);A.pop(i);PRINT.pop(n);g['S'].pop(n,None)
 link=box([55.2,14.2,55.2],[74.8,21.8,76.8])
 for xx in [60,70]:
  for zz in [60,72]:
   link-=socket(xx,zz,14.1,21.9,21.8)
   native('Timing connection 3L pin '+str(xx)+' '+str(zz),'6558',[xx,26,zz],('master' if xx==60 else 'slave'),axis=1);P[-1]['motion']='crosshead'
 emit(n,link,'connections',(1,-1),motion='crosshead')
