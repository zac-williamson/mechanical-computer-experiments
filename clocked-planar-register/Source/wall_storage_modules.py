"""Storage wear cartridges and independent bolt guides, with reusable frame seats.

Rail cartridge: bed at +Y rear face. Guide cheeks: bed at outer X face;
Frame-pin bores have 45° teardrop roofs in their actual print orientation.
Band anchor: front (-Y) face on bed, building toward +Y.
"""
import json,numpy as np,manifold3d as m,trimesh
from wall_flat_frame import project_back

def split_storage(g):
 P,A=g['P'],g['A'];box,cyl,solid,triangles=[g[k] for k in ['box','cyl','solid','triangles']]
 records=[];prints=[];obsolete=[]
 schedules=json.loads((g['OUT']/'Frame fixture schedule.json').read_text())
 def project(s,axis,bed,sign):
  # Map the chosen outward bed direction to +Y for the existing projection.
  import trimesh
  v=np.eye(3)[axis]*sign;T=trimesh.geometry.align_vectors(v,[0,1,0]);R=T[:3,:3]
  return project_back(s.transform(T[:3]),bed*sign).transform(np.c_[R.T,np.zeros(3)])
 def rail_bore(x,z,y):
  # One revolved cutter avoids coincident internal faces where the 45-degree
  # mouth meets the straight socket; there is no tiny horizontal ledge.
  t=trimesh.creation.revolve([[0,30.1],[2.5,30.1],[2.5,y-1.7],[3.3,y-.9],[3.3,y+.3],[0,y+.3]],sections=32)
  return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64))).rotate([-90,0,0]).translate([x,0,z])
 def tear_y(x,y0,y1,z,roof,r=2.5):
  # Horizontal pin bore; extend its roof in the print-up X direction.
  s=cyl(r,y0,y1,1,[x,0,z]);d=r/np.sqrt(2)
  pts=np.array([[x+roof*d,z-d],[x+roof*d,z+d],[x+roof*r*np.sqrt(2),z]])
  s+=m.CrossSection([pts],m.FillRule.EvenOdd).extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
  return s
 def emit(name,s,axis,sign):
  assert len([c for c in s.decompose() if c.volume()>.001])==1,(name,'disconnected',[(c.volume(),c.bounding_box()) for c in s.decompose()])
  g['emit'](name,s,module='bit',motion='fixed',color=(.42,.65,.60));prints.append(dict(part=name,axis=axis,sign=sign))
 for bank,dx,host,frame in [('master',0,'bit frame 0 removable fixture 3','bit coordinated chassis 0'),('slave',106,'bit frame 1 removable fixture 1','bit coordinated chassis 1')]:
  i=next(i for i,p in enumerate(P) if p['id']==host);old=solid(A[i]).translate([-dx,0,0])
  obsolete.append(host)
  previous=next(e for e in schedules['fixtures'] if e['part']==host)
  obsolete.extend(p['part'] for p in previous['fasteners']);schedules['fixtures'].remove(previous)
  # A complete replaceable rail and anti-rocking backing, rather than a seam
  # crossing the carriage travel. No obsolete legacy pin holes survive.
  left=-16.4 if bank=='slave' else -20
  rail=box([-19.8,17.05,9.6],[19.8,37.8,11.6])+box([left,32.8,6.8],[20,37.8,15.9])
  for x in [-6.8,6.8]:
   rail+=box([x-4.8,30.2,-4.8],[x+4.8,37.8,4.8])+box([x-3,32.8,4.7],[x+3,37.8,9.7])
  if bank=='slave':rail-=box([-21,28.1,6.7],[-16.4,38.3,11.7])
  # Preserve working guide cross-section but leave the sequencing cam's full
  # Y corridor open below Z54.3. Separate cheeks avoid unsupported bridging.
  working=(old^box([-18.85,14,54],[8.75,37.8,65])).slice(58).extrude(18.2).translate([0,0,46.2])
  working-=box([-40,25.3,40],[40,32.5,54.3])
  guides=[]
  for side,x,outer,sign,clip in [('left',-20.5,-25.3,1,[-40,-5.05]),('right',10.5,15.3,-1,[-5.05,40])]:
   s=working^box([clip[0],0,40],[clip[1],50,70])
   s+=box([x-4.8,33,44.2],[x+4.8,40.6,63.8])
   # Join each working cheek to its own rear mounting rail.
   if side=='left':s+=box([-20.5,33,46.2],[-14,40.6,64.4])
   else:s+=box([3.5,33,46.2],[10.5,40.6,64.4])
   s+=box([min(outer,outer+3*sign),14,46.2],[max(outer,outer+3*sign),40.6,64.4])
   s-=box([-40,25.3,40],[40,32.5,54.3])
   s=project(s,0,outer,-sign)
   for z in [49,59]:s-=tear_y(x,32.9,40.7,z,sign)
   guides.append(s)
  # A separately mounted, front-rooted hook can receive a closed band from
  # +Y on the bench. The retaining lip grows at 45 degrees in its +Y print.
  bx=-5.05
  anchor=(cyl(4.8,33,40.6,1,[-13.5,0,39.8])+cyl(4.8,33,40.6,1,[3.5,0,39.8])).hull()
  anchor+=box([bx-3,33,39.8],[bx+3,35.7,50])
  anchor+=cyl(2.4,33,35.7,1,[bx,0,50])+cyl(1.8,35.7,37.3,1,[bx,0,50])
  anchor+=m.Manifold.cylinder(.6,1.8,2.4,circular_segments=32).rotate([-90,0,0]).translate([bx,37.3,50])
  anchor+=cyl(2.4,37.9,38.9,1,[bx,0,50])
  for x in [-13.5,3.5]:anchor-=cyl(2.5,32.9,40.7,1,[x,0,39.8])
  names=[bank+' rail and backing cartridge',bank+' bolt guide left',bank+' bolt guide right',bank+' replaceable band anchor']
  for n,s,ax,sg in zip(names,[rail,*guides,anchor],[1,0,0,1],[-1,1,-1,1]):emit(n,s.translate([dx,0,0]),ax,sg)
  fi=next(i for i,p in enumerate(P) if p['id']==frame);base=solid(A[fi])
  base+=box([dx-29,44,-5],[min(dx+29,28.3) if dx==0 else dx+29,48.5,64.4])
  # Clear the new guide depth from the front of the base. A straight recess
  # open in -Y preserves rear-bed printing, with no internal roof.
  for ss in [*guides,anchor]:
   bb=np.array(ss.bounding_box()).reshape(2,3);lo=bb[0]+[dx-.2,-1,-.2];hi=bb[1]+[dx+.2,.4,.2]
   base-=box([lo[0],-50,lo[2]],[hi[0],min(hi[1],41.0),hi[2]])
  for n,s,pins,y in [(names[0],rail,[(-6.8,0),(6.8,0)],38),(names[1],guides[0],[(-20.5,49),(-20.5,59)],40.5),(names[2],guides[1],[(10.5,49),(10.5,59)],40.5),(names[3],anchor,[(-13.5,39.8),(3.5,39.8)],40.5)]:
   pinrows=[];j=next(j for j,p in enumerate(P) if p['id']==n);body=solid(A[j])
   for k,(x,z) in enumerate(pins):
    x+=dx
    # Broad rectangular frame sockets grow directly from the rear bed.
    base+=box([x-4.8,y+.2,z-4.8],[x+4.8,48.5,z+4.8])
    base-=cyl(2.5,y+.19,48.6,1,[x,0,z])
    inner_left=2 if n==names[2] else 0;inner_right=2 if n==names[1] else 0
    body+=(cyl(4.8,y-.2,y+.2,1,[x,0,z]) if n==names[3] else box([x-4.8,y-.2,z-4.8],[x+4.8,y+.2,z+4.8])+box([x-4.8-inner_left,y-.2,z-3],[x+4.8+inner_right,y+.2,z+3]))
    # Keep pin collar entirely outside the bores; chamfer it rather than
    # leaving an unsupported flat counterbore shoulder.
    base-=cyl(3.3,y-.91,y+.9,1,[x,0,z])
    if n==names[0]:pass # Cut both sockets after projecting the unperforated rail.
    elif n==names[3]:
     body-=cyl(2.5,32.9,y+.3,1,[x,0,z])+cyl(3.3,y-.9,y+.3,1,[x,0,z])
    else:
     roof=1 if n==names[1] else -1
     body-=tear_y(x,32.9,y+.3,z,roof)
     body-=tear_y(x,y-.9,y+.3,z,roof,3.3)
    pn=n+' mounting pin '+str(k+1);g['native'](pn,'2780',[x,y,z],axis=1,module='bit');pinrows.append(dict(part=pn,centre_mm=[x,y,z]))
   if n==names[0]:
    body=project_back(body,y+.2)
    for x,z in pins:
     x+=dx;body-=rail_bore(x,z,y)
   if n==names[0] and bank=='slave':
    relief=box([-40,28.1,6],[-16.4,39,17])+m.CrossSection([np.array([[-40,24.7],[-19.8,24.7],[-16.4,28.1],[-40,28.1]])],m.FillRule.EvenOdd).extrude(11).translate([0,0,6])
    body-=relief.translate([dx,0,0])
   body=sum((c for c in body.decompose() if c.volume()>.01),m.Manifold())
   A[j]=triangles(body);P[j]['vertices']=len(A[j]);schedules['fixtures'].append(dict(part=n,frame=frame,fasteners=pinrows,fastener_spacing_mm=float(np.linalg.norm(np.subtract(pins[0],pins[1]))),interface_y_mm=y,mount_pad_width_mm=9.6,collar_relief_radius_mm=3.3,minimum_collar_ligament_mm=1.5,seating_plane_y_mm=y+.2,seating_land_radius_mm=4.8,seating_gap_mm=0,pin_roof_axis=0 if n in names[1:3] else None,pin_roof_sign=(1 if n==names[1] else -1) if n in names[1:3] else None))
  A[fi]=triangles(base);P[fi]['vertices']=len(A[fi])
  # Lower band wrap moves -0.5 mm along Y, from Y37.3 to Y36.8;
  # the bolt's upper wrap remains at Y37.3.
  ei=next(i for i,p in enumerate(P) if p['id']==bank+' return band')
  A[ei][:,1]-=.5*np.clip((58.6-A[ei][:,2])/8.6,0,1)

 for i in reversed(range(len(P))):
  if P[i]['id'] in obsolete:P.pop(i);A.pop(i)
 (g['OUT']/'Frame fixture schedule.json').write_text(json.dumps(schedules,indent=2))
 (g['OUT']/'Storage module schedule.json').write_text(json.dumps(dict(connections=records,print_orientations=prints,modules=['master','slave'],replaced_parts=obsolete),indent=2))
