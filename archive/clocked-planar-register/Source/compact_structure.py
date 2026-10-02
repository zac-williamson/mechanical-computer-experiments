"""Frame and common-fork sequencer candidate. Geometry must pass audits before release."""
from pathlib import Path
import numpy as np
import trimesh, manifold3d as m

def build(parts,arrays,add,native,lookup,raw):
 def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
 def cyl(r,a,b,axis,c):
  s=m.Manifold.cylinder(b-a,r,circular_segments=48)
  if axis==0:s=s.rotate([0,90,0])
  if axis==1:s=s.rotate([-90,0,0])
  p=list(c);p[axis]=a;return s.translate(p)
 def solid(a):
  t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
 def mesh(s):
  q=s.simplify(.001).to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
  if not t.is_watertight:
   q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
  if not t.is_watertight:
   # Boolean coplanar junctions can leave zero-area triangles after welding.
   # Remove only duplicate/collapsed faces; never fill a missing surface.
   t.update_faces(t.unique_faces());t.update_faces(t.nondegenerate_faces(height=1e-7));t.remove_unreferenced_vertices()
  if not t.is_watertight:raise ValueError('Non-watertight generated part')
  if len(t.split())!=1:raise ValueError('Disconnected generated part '+str([(round(v.volume,3),v.bounds.tolist()) for v in t.split()]))
  return t.triangles.reshape(-1,3)
 def get(name):return solid(arrays[next(i for i,p in enumerate(parts) if p['id']==name)])
 def replace(name,s):
  i=next(i for i,p in enumerate(parts) if p['id']==name);arrays[i]=mesh(s)
 def emit(name,s,motion='fixed',bank='frame',color=(.48,.55,.47)):

  try:a=mesh(s)
  except ValueError as e:raise ValueError(name+' '+str(e)) from e
  add(name,a,list(color),'printed',motion=motion,bank=bank)
 def oldsolid(name):
  p=lookup[name];return solid(raw[p['offset']//3:p['offset']//3+p['vertices']])
 def xz(poly,y0,y1):return m.CrossSection([np.array(poly)]).extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
 def link(a,b,r,y0,y1):return (cyl(r,y0,y1,1,[a[0],0,a[1]])+cyl(r,y0,y1,1,[b[0],0,b[1]])).hull()
 # Drop explicitly labelled layout-only lock blocks.
 for i in reversed(range(len(parts))):
  if parts[i]['kind']=='envelope':parts.pop(i);arrays.pop(i)
 # Local rear clearance for the larger, equal-speed idlers. Strengthen and
 # raise the rear lower joining pin, rather than cutting through its bearing.
 for bank,x,side in [('master',0,-16),('slave',106,16)]:
  for short,xa,xb in [('Carriage fork and roof',.2,7.8),('Right carriage bearing support',8,15.6)]:
   name=bank+' '+short;s=get(name)
   s+=box([x+xa,20.2,.0],[x+xb,27.8,8.7])
   s-=cyl(2.5,x+xa-.1,x+xb+.1,0,[0,24,5.3])
   s-=cyl(9.25,x+side-8,x+side+8,0,[0,10.2+np.sqrt(192),-8])
   replace(name,s)
  name=bank+' Carriage support pin 1 2.0';i=next(i for i,p in enumerate(parts) if p['id']==name);arrays[i][:,2]+=1.8;parts[i]['local_pin_shift_z']=1.8
 # Fixed cheeks preserve working bearing/contact faces, cropped beyond the
 # working region and mounted above the moving roof on rear-reaching posts.
 # CLOCK needs the planar worm carriage, not the old clock assembly's
 # extended fork/drive tower. Remove only its unused output-clutch portion.
 for short in ['Carriage fork and roof','Right carriage bearing support']:
  name='clock '+short
  replace(name,(get(name)^box([0,-5,-85],[80,40,-24]))-cyl(9.2,24,40,0,[0,10.2,-16]))
 mounts=[];base=m.Manifold()
 for bank,srcx,x,z,flip in [('master',0,0,0,False),('slave',160,106,0,False),('write',-85,-76,-16,True),('clock',75,40,-16,True)]:
  srcz=82.4 if bank in ['write','clock'] else 0
  for short in ['Front bearing cheek','Rear bearing cheek']:
   p=lookup[bank+' '+short];a=raw[p['offset']//3:p['offset']//3+p['vertices']].copy()-[srcx,0,srcz]
   # Keep both working pivot bosses (right pivot X=13.1923), but
   # remove the obsolete outer mounting tail beyond X=22.
   s=solid(a)^box([-8,-5,16],[22,25,39])
   rear=short.startswith('Rear')
   yy0,yy1=(14.8,18.6) if rear else (1.8,5.6)
   s+=box([-27.8,yy0,24.6],[-1.3,yy1,38.0])
   for zz in [28.4,35.6]:
    s+=cyl(3.8,10 if rear else 1.8,18.6 if rear else 9.6,1,[-24,0,zz])
    s-=cyl(2.5,1.7,18.7,1,[-24,0,zz])
   if rear:
    s+=box([-27.8,18.4,24.6],[-20.2,37.8,38.0])
    for zz in [28.4,35.6]:s-=cyl(2.5,29.9,37.9,1,[-24,0,zz])
   if rear:
    # Restore the actuator's essential neutral-return anchor. The cropped
    # legacy cheek had removed it. Keep its line through the lever pivot,
    # shortening only the unloaded anchor reach to fit the compact frame.
    ax,az={'write':(30.192323604,29.295115365),'clock':(29.192323604,29.461782031)}.get(bank,(31.192323604,29.128448698))
    s+=link((13.192323604,32.128448698),(ax,az),2.8,14.8,18.6)
    lip=2.8 if bank=='clock' else 3.2
    s+=cyl(lip,11.2,18.6,1,[ax,0,az])+cyl(2,9.2,11.2,1,[ax,0,az])+cyl(lip,8,9.2,1,[ax,0,az])
   # All added ribs must leave the original working bores open. Recut
   # after unions, including the anchor web through the pivot boss.
   for hx,hz in [(0,24),(13.192323604,32.128448698)]:s-=cyl(2.65,1.7,18.7,1,[hx,0,hz])
   if flip:s=s.rotate([0,180,0])
   s=s.translate([x,0,z]);emit(bank+' '+short,s,bank=bank)
  for zz in [28.4,35.6]:
   px=x+24 if flip else x-24;pz=z-zz if flip else z+zz
   mounts.append((px,pz));native(bank+' cheek frame pin '+str(zz),'2780',[px,38,pz],1)
   native(bank+' cheek joining pin '+str(zz),'2780',[px,9.8,pz],1)
 # Preserve each original linear guide profile; support it from the rear.
 mux=Path(__file__).resolve().parents[2]/'planar-register/work/register-mux-reference/multiplexer/Common baseboard.stl'
 original_track=solid(trimesh.load(mux).triangles.reshape(-1,3))^box([-19.8,15.5,9],[19.8,32.8,12.2])
 for bank,x,z,flip in [('master',0,0,False),('slave',106,0,False),('write',-76,-16,True),('clock',40,-16,True)]:
  track=original_track
  if bank=='slave':track=track^box([-16,15.4,8.9],[20,32.9,12.3])
  supports=[(-15.8,-8.2),(8.2,15.8)] if bank in ['master','slave'] else [(-27.8,-19),(19,27.8)] if bank=='clock' else [(-23.8,-16),(16,23.8)]
  if bank=='clock':
   for a,b in [(-27.8,-16),(16,27.8)]:track+=box([a,30.2,9],[b,32.8,12.2])
  for a,b in supports:
   track+=box([a,32.6,9],[b,37.8,12.2])+box([a,30.2,6.8],[b,37.8,14.4])
   xx=(a+b)/2;track-=cyl(2.5,30.1,37.9,1,[xx,0,10.6])
   px=x-xx if flip else x+xx;pz=z-10.6 if flip else z+10.6
   mounts.append((px,pz));native(bank+' track mount '+str(xx),'2780',[px,38,pz],1)
  if flip:track=track.rotate([0,180,0])
  emit(bank+' carriage track',track.translate([x,0,z]),bank=bank)
 # Small end-bearing walls are generated from explicit shaft locations.
 wall_data=[('M output left',-24,[(10.2,0)]),('M output right',26,[(10.2,0)]),('Q output left',82,[(10.2,0)]),('Q output right',132,[(10.2,0)]),
 ('M worm left',-28,[(10.2,16)]),('M worm right',24,[(10.2,16)]),('Q worm right',130,[(10.2,16)]),('Q gate left',30.2,[(10.2,16)]),
 ('WRITE worm left',-104,[(10.2,-32)]),('WRITE worm right',-48,[(10.2,-32)]),('CLK worm left',12,[(10.2,-32)]),('CLK worm right',76,[(10.2,-32)]),
 ('D header',-104,[(18.2,0)]),('D header inner',-94,[(18.2,0)]),('D transfer left',-76,[(10.2,0)]),('D transfer right',-68,[(10.2,0)]),
 ('POWER bus',68,[(10.2,-16)]),('POWER input',-104,[(10.2-np.sqrt(192),-24)]),('POWER input right',68,[(10.2-np.sqrt(192),-24)]),
 ('Feedback left',-100,[(10.2-np.sqrt(192),-8)]),('Feedback right',144,[(10.2-np.sqrt(192),-8)]),
 ('CLK external left',-104,[(10.2-np.sqrt(551),-37)]),('CLK external right',-20,[(10.2-np.sqrt(551),-37)])]
 wall_data += [('M idler left',-24,[(10.2+np.sqrt(192),-8)]),('M idler right',24,[(10.2+np.sqrt(192),-8)]),
  ('Q idler left',114,[(10.2+np.sqrt(192),-8)]),('POWER bus left',24,[(10.2,-16)]),
  ('WRITE output left',-102.4,[(10.2,-16)]),('WRITE output right',-28,[(10.2,-16)]),
  ('Master data left',-50,[(10.2,0)]),('Master data right',-38,[(10.2,0)])]
 wall_solids=[];wall_pin_candidates=[]
 for name,x,axes in wall_data:
  width=2.4 if name=='Q gate left' else 3.2 if name in ['Q gate left','D transfer left','D transfer right','M output right','Q output right','Feedback right','WRITE output left','Master data left','Master data right','D header inner'] else 6.8 if name in ['M worm left','M worm right','M idler left','M idler right','Q idler left','POWER bus left','WRITE output right'] else 7.6
  s=m.Manifold();zs=[]
  routezs=[]
  for y,z in axes:
   route=z
   if abs(z-16)<.01:route=4 if x<0 else 34
   def yzdisc(yy,zz):return cyl(5.5,x-width/2,x+width/2,0,[0,yy,zz])
   # Route the support around the shared moving bar, not through it.
   yy=25
   # Carry the shaft support behind the lever plane before changing Z.
   # A diagonal starting at the bore cuts through the reaction mechanism.
   s+=(yzdisc(y,z)+cyl(4.5,x-width/2,x+width/2,0,[0,25,z])).hull()
   s+=(cyl(4.5,x-width/2,x+width/2,0,[0,25,z])+cyl(4.5,x-width/2,x+width/2,0,[0,25,route])).hull()
   s+=box([x-width/2,yy,route-4.5],[x+width/2,37.8,route+4.5])
   s-=cyl(2.65,x-width/2-.1,x+width/2+.1,0,[0,y,z]);zs.append(route)
  pin_zs=([33,41.8] if zs==[34] else [-5,11.6] if zs==[4] else [-31,-21] if name=='WRITE worm right' else [-17,-7] if name in ['POWER bus left','WRITE output left'] else [-9,1] if name=='D header inner' else [-19,-9] if name in ['D transfer left','D transfer right','Master data left','Master data right','Q output left'] else [min(zs)-9,max(zs)+9])
  if name=='POWER bus left':pin_zs=[-9,0]
  if name=='CLK worm left':pin_zs=[-41,-15]
  for z in pin_zs:
   px=12 if name=='POWER bus left' else x
   if px!=x:s+=box([min(px,x)-3.8,30.2,z-3.8],[max(px,x)+3.8,37.8,z+3.8])
   s+=box([x-3.8,30.2,z-3.8],[x+3.8,37.8,z+3.8])+box([x-width/2,30.2,min(z,min(zs))],[x+width/2,37.8,max(z,max(zs))])
   wall_pin_candidates.append((name,px,z))
  wall_solids.append((min(x-max(width/2,3.8),8.2 if name=='POWER bus left' else x),x+max(width/2,3.8),name,s))
 # Consolidate overlapping wall planes; every crossing shaft gets an
 # actual through bearing bore. This is a multi-axis support, not a trimmed wall.
 groups=[]
 for lo,hi,name,s in sorted(wall_solids,key=lambda q:q[0]):
  if groups and lo<groups[-1][1]:
   g=groups[-1];g[1]=max(g[1],hi);g[2].append(name);g[3]+=s
  else:groups.append([lo,hi,[name],s])
 for i,(lo,hi,names,s) in enumerate(groups):
  for p in parts:
   if p.get('axis')!=0 or p.get('lego_part') not in ['3705','3706','3707','3708','3737','32073','32062','4519','60485','23948','44294','50450','50451']:continue
   a,b=p['bounds']
   if min(hi,b[0])-max(lo,a[0])>0:
    y,z=p['centre'][1:];s-=cyl(2.65,lo-.1,hi+.1,0,[0,y,z])
  # The front feedback support passes the WRITE output bush, not just
  # its axle. Provide the full rotating-bush clearance in that web.
  if 'Feedback left' in names:s-=cyl(3.9,-100.3,-95.7,0,[0,10.2,-16])
  if 'WRITE output left' in names:s-=cyl(3.9,-108.3,-104.1,0,[0,10.2,-16])
  if 'D header' in names:s-=cyl(3.9,-100.2,-95.6,0,[0,18.2,0])
  # X extents alone do not imply that supports touch in Y/Z. Keep
  # disconnected supports as separately pinned printable parts.
  for j,piece in enumerate(s.decompose()):
   # Pin each actual connected support twice. Choose separated mount
   # positions, rather than piling overlapping pins into merged walls.
   q=piece.to_mesh64();tm=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
   candidates=list(dict.fromkeys((px,pz) for name,px,pz in wall_pin_candidates if name in names))
   candidates=[c for c in candidates if tm.contains([[c[0],34,c[1]]])[0] and all(np.linalg.norm(np.array(c)-d)>=6.8 for d in mounts)]
   candidates=[(px,pz) for px,pz in candidates if px+3.8<=20.2 or px-3.8>=59.8 or pz+3.8<=-32.4 or pz-3.8>=29.2]
   pairs=[(a,b) for k,a in enumerate(candidates) for b in candidates[k+1:] if np.linalg.norm(np.array(a)-b)>=6.8]
   if not pairs:raise ValueError('No two clear frame pins for bearing support '+str(i)+'.'+str(j)+' names='+str(names)+' candidates='+str(candidates))
   chosen=max(pairs,key=lambda p:np.linalg.norm(np.array(p[0])-p[1]))
   for k,(px,pz) in enumerate(chosen):
    piece-=cyl(2.51,29.9,37.9,1,[px,0,pz]);mounts.append((px,pz))
    native('Bearing support '+str(i)+'.'+str(j)+' mount '+str(k),'2780',[px,38,pz],1)
   emit('Bearing support '+str(i)+'.'+str(j)+' — '+' + '.join(names),piece)
 # Compact bolts retain the original tip and roller bearing, with a centred
 # rear band bridge ABOVE the roller and cam. All bushes/axles remain LEGO.
 for bank,x in [('master',0),('slave',106)]:
  bx=x-5.05
  bolt=oldsolid('master Direct lock bolt')^box([-30,-5,40],[20,40,59.8])
  bolt+=box([-9.05,24.8,57.5],[-1.05,35.8,59.8])
  for y0,y1,r in [(35.7,36.7,1.8),(36.7,37.9,1.2),(37.9,38.9,1.8)]:bolt+=cyl(r,y0,y1,1,[-5.05,0,58.6])
  emit(bank+' lock bolt',bolt.translate([x,0,0]),'bolt',bank,(.8,.64,.18))
  guide=box([bx-10,14,45.5],[bx+10,40.4,46.2])+box([bx-10,14,45.5],[bx+10,24.8,49.2])
  guide-=box([bx-2.25,17.7,45.4],[bx+2.25,22.3,49.3])
  for xa,xb in [(-10,-6.4),(6.4,10)]:
   guide+=box([bx+xa,14,45.5],[bx+xb,40.4,64.4])
  guide-=box([bx-10.1,25.2,46.4],[bx+10.1,35.6,54.4])
  for xa,xb in [(-6.4,-4.6),(4.6,6.4)]:
   guide+=box([bx+xa,14,52],[bx+xb,17.6,64.4])+box([bx+xa,25.6,54.4],[bx+xb,29.2,64.4])
  for xx in [bx-10,bx+10]:
   guide+=box([min(xx,bx)-.1,36,45.5],[max(xx,bx)+.1,40.4,49.2])
   guide+=box([xx-3.8,30.2,36.2],[xx+3.8,37.8,46.2]);guide-=cyl(2.5,30.1,37.9,1,[xx,0,40])
   mounts.append((xx,40));native(bank+' guide frame pin '+str(xx),'2780',[xx,38,40],1)
  guide+=box([bx-3,35.7,45.5],[bx+3,36.5,50])
  for y0,y1,r in [(35.7,36.7,2.4),(36.7,37.9,1.8),(37.9,38.9,2.4)]:guide+=cyl(r,y0,y1,1,[bx,0,50])
  guide=guide^box([bx-30,-5,0],[bx+30,37.8,80])
  for y0,y1,r in [(35.7,36.7,2.4),(36.7,37.9,1.8),(37.9,38.9,2.4)]:guide+=cyl(r,y0,y1,1,[bx,0,50])
  guide-=cyl(2.7,36.7,37.9,1,[bx,0,50])-cyl(1.8,36.6,38,1,[bx,0,50])
  emit(bank+' compact bolt guide',guide,bank=bank)
  native(bank+' roller axle','32062',[bx,21.6,53.3],1)
  for yy in [15.6,27.6]:native(bank+' roller bush '+str(yy),'32123a',[bx,yy,53.3],1)
  # Closed elastic return loop seated in the two real anchor grooves.
  # This is purchased elastic, not a printed part. Preload needs measurement.
  from shapely.geometry import MultiPoint
  circles=[(bx+r*np.cos(a),z+r*np.sin(a)) for z,r in [(50,2.2),(58.6,1.6)] for a in np.linspace(0,2*np.pi,96,endpoint=False)]
  path=np.array(MultiPoint(circles).convex_hull.exterior.coords[:-1]);vv=[];ff=[]
  for j,(xx,zz) in enumerate(path):
   tangent=path[(j+1)%len(path)]-path[(j-1)%len(path)];tangent/=np.linalg.norm(tangent)
   n=np.array([-tangent[1],0,tangent[0]])
   for a in np.linspace(0,2*np.pi,12,endpoint=False):vv.append(np.array([xx,37.3,zz])+.4*(n*np.cos(a)+np.array([0,1,0])*np.sin(a)))
  for j in range(len(path)):
   for k in range(12):
    a=j*12+k;b=j*12+(k+1)%12;c=((j+1)%len(path))*12+k;d=((j+1)%len(path))*12+(k+1)%12
    ff.extend([(a,b,c),(b,d,c)])
  band=trimesh.Trimesh(vv,ff,process=True)
  add(bank+' return band',band.triangles.reshape(-1,3),[.48,.2,.52],'elastic',motion='elastic',bank=bank)
 # Passive forks wait for dog alignment; the bar positively withdraws them.
 # Four millimetres of one-sided lost motion cannot delay disengagement.
 rail=box([-88,31,16],[150,34.6,24])
 for bank,gx in [('master_gate',-60),('slave_gate',60)]:
  bias=1 if bank=='master_gate' else -1
  for short in ['Carriage fork and roof','Right carriage bearing support']:
   src=oldsolid('master '+short)^box([-20,-5,-1],[20,28,8])
   if short=='Carriage fork and roof':
    # Offset only the clutch finger; retain the compact rear guides and
    # pin-joined body positions. This preserves the swept clearance to the
    # storage carriage while the native gear returns beside its connector.
    dx=-6*bias
    front=(src^box([-20,-5,-1],[20,14.05,8])).translate([dx,0,0])
    rear=src^box([-20,14.0,-1],[20,28,8])
    bridge=(box([dx-7.3,13.75,6.5],[dx+7.3,14.05,8])+box([-15.6,17.95,6.5],[7.8,18.25,8])).hull()
    bridge-=box([7.9,14.55,-2],[25,29,9])
    # The gear now sits beside its connector. Clear its whole rotating
    # envelope from the rear shoulder, while keeping the offset finger
    # outside the gear's axial sweep and intact around the clutch groove.
    gear_relief=cyl(9.4,1.3 if bias>0 else -20,20 if bias>0 else -1.3,0,[0,10.2,0])
    src=front+((rear+bridge)-gear_relief)

   xx=-10 if short=='Carriage fork and roof' else 12
   # Pins are fixed in each fork. Slots in the bar permit only a retreat
   # from engagement; the opposite slot end is the withdrawal shoulder.
   src+=box([xx-3.8,20.2,-6.6],[xx+3.8,28,8.2])
   for zz in [-2.8]:
    src-=cyl(2.5,20.1,28.1,1,[xx,0,zz])+cyl(3.3,27,28.1,1,[xx,0,zz])
   # Short front-facing band anchors avoid the pin shafts and rear frame.
   # The rectangular stems are wider in the loaded X direction.
   ax=-12 if xx<0 else 11;az=7.8
   # The two halves are rigidly pin-joined. One bias loop per fork avoids
   # the right-hand anchor sweeping into the adjacent shaft bearing web.
   if xx<0:
    src+=box([ax-2.5,20.2,6.9-16],[ax+2.5,28,11.5-16])
    src+=cyl(2.5,18.2,18.9,1,[ax,0,az-16])+cyl(1.2,18.8,20.3,1,[ax,0,az-16])
    src+=box([-15.6,18.2,7.3-16],[ax-2.4,20.3,8.3-16])
   for zz in [-2.8]:src-=cyl(2.5,20.1,28.1,1,[xx,0,zz])+cyl(3.3,27,28.1,1,[xx,0,zz])
   src-=cyl(2.5,.1,16.3,0,[0,24,3.5])+cyl(3.3,7.1,9.3,0,[0,24,3.5])
   src=src.set_tolerance(.0001)
   emit(bank+' '+short,src.translate([gx,0,16]),'gate-fork',bank,(.72,.4,.19))
   if xx<0:
    anchor=gx+ax+6.5*bias
    rail+=box([anchor-2.5,20.2,6.9],[anchor+2.5,28.2,8.9])+box([min(anchor,gx+xx)-2.5,28.2,6.9],[max(anchor,gx+xx)+2.5,29.8,10.1])
    rail+=cyl(2.5,18.2,18.9,1,[anchor,0,az])+cyl(1.2,18.8,20.3,1,[anchor,0,az])
    from compact_elastic import fork_band
    add(bank+' fork bias band '+str(xx),fork_band(bank,xx,0,0),[.48,.2,.52],'elastic',motion='fork-band',bank=bank,fork_x=xx)
   rail+=box([gx+min(xx,xx-4*bias)-3.8,28.2,9.4],[gx+max(xx,xx-4*bias)+3.8,35.8,24.2])
   for zz in [13.2]:
    native(bank+' crosshead pin '+str(xx)+' '+str(zz),'2780',[gx+xx,28,zz],1)
    parts[-1].update(motion='gate-fork',bank=bank)
  native(bank+' fork joining pin','2780',[gx+8.2,24,19.5],0)
  parts[-1].update(motion='gate-fork',bank=bank)
 # Shared cam geometry: the operation model uses circular contact with this outline.
 from compact_cam import outline
 profile=outline().tolist()
 poly=[[-88,47],[155,47]]+profile[::-1]
 rail+=xz(poly,25.6,31)+box([-88,30,46.6],[155,32.2,54])
 for a,b in [(-88,-84),(151,155)]:rail+=box([a,25.6,16],[b,34.6,50])
 for xx in [-46,57]:rail+=box([xx-2,31,16],[xx+2,34.6,50])
 for gx in [-60,60]:
  bias=1 if gx<0 else -1
  for xx in [gx-10,gx+12]:
   for zz in [13.2]:rail-=link((xx,zz),(xx-4*bias,zz),2.7,28.1,35.9)+link((xx,zz),(xx-4*bias,zz),3.4,28.1,29)
 rail-=box([-76,25.5,15.8],[-71.9,28.2,24.2])
 # Compact clock connection: a shallow post joined across X, not another
 # 8 mm rear Y layer. Working shaft axes and amplifier lever stay fixed.
 rail+=box([30,31,13.2],[39.8,43.8,28.8])
 post=box([30,44.2,-32],[50,47.8,24])-link((40,-27.7),(40,-26),3.9,44.1,47.9)
 post+=box([40.2,36.2,13.2],[50,47.8,28.8])
 for zz in [17,25]:
  rail-=cyl(2.5,31.9,39.9,0,[0,40,zz]);post-=cyl(2.5,40.1,48.1,0,[0,40,zz])
  native('Crosshead post pin '+str(zz),'2780',[40,40,zz],0)
 emit('Common fork and cam bar',rail,'crosshead','clock',(.72,.4,.19));emit('Clock output post',post,'crosshead','clock',(.72,.4,.19))
 # Short captured guides around the thinner, taller main bar.
 for xx in [0,106]:
  guide=box([xx-4,28.6,12.6],[xx+4,37.8,30])
  guide-=box([xx-4.1,30.6,15.6],[xx+4.1,35,24.4])
  guide+=box([xx-3.8,35.4,5.4],[xx+3.8,37.8,13])
  for zz in [5.4,31]:
   guide+=box([xx-3.8,33.2 if zz==5.4 else 30.2,zz-3.8],[xx+3.8,37.8,zz+3.8])
   guide-=cyl(2.5,30.1,37.9,1,[xx,0,zz]);mounts.append((xx,zz))
   native('Crosshead guide pin '+str(xx)+' '+str(zz),'2780',[xx,38,zz],1)
  emit('Captured crosshead guide '+str(xx),guide)
 # Split amplifier: integral cam shoes remove the two highly loaded
 # cantilevered LEGO follower axles. Both halves print on their joint faces.
 # Cam contact centres and travel remain the original mechanism's datums.
 front=link((40,-56),(40,-41),6,36.2,39.6)
 rear=link((40,-56),(40,-26),6,40,43.8)
 # Preserve clearance to the original pivot faces and frame.
 front-=box([20,36.1,-63],[60,38.2,-48.3])
 rear-=box([20,41.8,-63],[60,44,-48.3])
 front+=cyl(5,33.39,36.3,1,[40,0,-44])+cyl(3.6,28,32.01,1,[40,0,-44])
 front+=m.Manifold.cylinder(1.4,3.6,5,circular_segments=64).rotate([-90,0,0]).translate([40,32,-44])
 rear+=cyl(3.6,43.7,48,1,[40,0,-26])
 for xx in [35.5,44.5]:
  front+=cyl(3.8,32,39.6,1,[xx,0,-41])
  rear+=cyl(3.8,40,47.6,1,[xx,0,-41])
  front-=cyl(2.5,31.9,39.7,1,[xx,0,-41])
  rear-=cyl(2.5,39.9,47.7,1,[xx,0,-41])
  native('Clock amplifier joining pin '+str(xx),'2780',[xx,39.8,-41],1)
  parts[-1]['motion']='amplifier'
 front-=cyl(2.65,36.1,40,1,[40,0,-56])
 rear-=cyl(2.65,39.9,44,1,[40,0,-56])
 emit('Clock amplifier front and input shoe',front,'amplifier','clock',(.72,.4,.19))
 emit('Clock amplifier rear and output shoe',rear,'amplifier','clock',(.72,.4,.19))
 name='clock Carriage fork and roof';s=get(name)
 s+=box([34,27.6,-49],[46,32,-39]);s-=link((40,-44.7),(40,-44),3.9,25.9,32.1);replace(name,s)
 # The pivot is now carried between two grounded bearings; no follower axle
 # transmits the 200 N ideal stall force as a short cantilever.
 native('Clock amplifier pivot','4519',[40,38,-56],1)
 for yy in [32,48]:native('Clock pivot bush '+str(yy),'32123a',[40,yy,-56],1)
 cradle=link((20,-56),(52,-56),5,34.2,37.8)-cyl(2.65,34.1,37.9,1,[40,0,-56])
 for xx,zz in [(20,-49),(52,-54)]:
  cradle+=link((xx,-56),(xx,zz),3.8,34.2,37.8)+cyl(3.8,34.2,41.8,1,[xx,0,zz]);cradle-=cyl(2.5,34.1,41.9,1,[xx,0,zz])
  base+=link((xx,-56),(xx,zz),3.8,42.2,49.8);base-=cyl(2.5,42.1,49.9,1,[xx,0,zz])
  native('Clock pivot cradle mount '+str(xx),'2780',[xx,42,zz],1)
 emit('Clock front pivot cradle',cradle)
 base+=cyl(6,42.2,46,1,[40,0,-56]);base-=cyl(2.65,42.1,46.1,1,[40,0,-56])+cyl(3.8,46,50.1,1,[40,0,-56])
 # Open rear frame, entirely within the input/output ends. Mounting holes
 # are generated from actual support datums rather than moved legacy walls.
 for z in [-56,-16,40,56]:base+=box([-108,38.2,z-3.8],[158,46,z+3.8])
 for x,z in mounts:
  near=min([-56,-16,40,56],key=lambda v:abs(v-z))
  base+=box([x-3.8,38.2,min(z,near)-3.8],[x+3.8,46,max(z,near)+3.8]);base-=cyl(2.5,38.1,46.1,1,[x,0,z])
 # Frame rail supports without covering the XZ view.
 for x in [-104,-30,68,154]:base+=box([x-3.8,38.2,-59.8],[x+3.8,46,59.8])
 # Full window for the translating clock post, axle and bushes.
 base-=box([20.2,38.1,-32.4],[59.8,46.1,29.2])
 # Split at X=24. Native pins, no screws; add joints before partitioning.
 for z in [49,56]:
  base+=box([16,38.2,z-4],[32,46,z+4]);base-=cyl(2.5,15.9,32.1,0,[0,42,z]);native('Frame joint pin '+str(z),'2780',[24,42,z],0)
 for xx,zz in [(20,-49),(52,-54)]:base-=cyl(2.51,42.1,49.9,1,[xx,0,zz])
 base-=cyl(4.15,38.1,42.15,1,[20,0,-49])
 # Re-cut mount holes after all rails are joined: later unions must not
 # refill an earlier pin bore at a crossing of two frame members.
 for px,pz in mounts:base-=cyl(2.51,38.1,46.1,1,[px,0,pz])
 base-=cyl(2.65,42.1,46.1,1,[40,0,-56])+cyl(3.8,46,50.1,1,[40,0,-56])
 base-=box([24.0,38.1,-62.4],[55.8,42.21,-19.6])
 # The former integral rear pivot support left a 31 mm unsupported ledge
 # above the amplifier. Manufacture this bridge flat on its Y=42.2 face.
 # Two non-collinear X pins locate it, with full-length sockets at each end.
 # Neither the pivot bore nor the moving amplifier is repositioned.
 rear_bridge=base^box([24.2,42.2,-63],[55.8,50,-48.0])
 base-=box([24.0,38.1,-63],[56.0,50.1,-48.0])
 for xx,zz in [(24,-56),(56,-59)]:
  inside=(xx+.2,xx+7.8) if xx==24 else (xx-7.8,xx-.2)
  outside=(xx-7.8,xx-.2) if xx==24 else (xx+.2,xx+7.8)
  rear_bridge+=box([inside[0],42.2,zz-3.8],[inside[1],49.8,zz+3.8])
  base+=box([outside[0],38.2,zz-3.8],[outside[1],49.8,zz+3.8])
  bore=cyl(2.5,xx-7.9,xx+7.9,0,[0,46,zz])
  rear_bridge-=bore;base-=bore
  native('Rear pivot bridge pin '+str(xx),'2780',[xx,46,zz],0)
 # Carry bending around the rear retaining bush, within the existing
 # Y=49.8 envelope. The 45-degree transition prints without a new ledge.
 rear_bridge+=cyl(6.8,46,49.8,1,[40,0,-56])
 rear_bridge+=box([24.2,46,-59.8],[55.8,49.8,-52.2])
 rear_bridge+=m.Manifold.cylinder(.8,6,6.8,circular_segments=48).rotate([-90,0,0]).translate([40,45.2,-56])
 rear_bridge-=cyl(2.51,42.1,49.9,1,[52,0,-54])
 rear_bridge-=cyl(2.65,42.1,46.1,1,[40,0,-56])+cyl(3.9,46,50.1,1,[40,0,-56])
 emit('Clock rear pivot bearing bridge',rear_bridge)
 # The new left socket overlaps the front cradle's mounting boss envelope.
 # Restore that intentional clearance after adding the socket material.
 base-=cyl(4.15,38.1,42.15,1,[20,0,-49])
 for bx in [-5.05,100.95]:
  base-=box([bx-5,38.1,56.4],[bx+5,46.1,64.6])
  base-=box([bx-3,38.1,47.2],[bx+3,46.1,52.8])
 for name,xa,xb in [('left',-120,23.8),('right',24.2,166)]:emit('Compact frame '+name,base^box([xa,-30,-85],[xb,65,68]))
 # The clock track and its neighbouring shaft pedestals occupy the same
 # fixed structural volume. Manufacture them as one grounded support module,
 # rather than three intersecting printed pieces.
 clock_track=get('clock carriage track');merged=['clock carriage track']
 for p in parts:
  if p['kind']=='printed' and p['id'].startswith('Bearing support'):
   candidate=get(p['id'])
   if (clock_track^candidate).volume()>.01:
    clock_track+=candidate;merged.append(p['id'])
 if len(merged)>1:
  for p in parts:
   if p.get('lego_part')=='2780' and p.get('axis')==1 and abs(p.get('centre',[0,0,0])[1]-38)<.001:
    px,_,pz=p['centre'];clock_track-=cyl(2.51,29.9,37.9,1,[px,0,pz])
  for i in reversed(range(len(parts))):
   if parts[i]['id'] in merged:parts.pop(i);arrays.pop(i)
  emit('Clock track and shaft support module',clock_track)
 # A LEGO friction pin's centre collar is wider than its insertion ribs.
 # Recess only confirmed mating bores, never an arbitrary intersecting part.
 pin_meshes=[(p,a) for p,a in zip(parts,arrays) if p.get('lego_part')=='2780']
 for part_index,(p,a) in enumerate(zip(parts,arrays)):
  if p['kind']!='printed':continue
  tm=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  cutters=[]
  for pin,pa in pin_meshes:
   axis=pin.get('axis',int(np.argmax(np.ptp(pa,axis=0))))
   c=np.array(pin.get('centre',(pa.min(0)+pa.max(0))/2),dtype=float)
   lo=a.min(0);hi=a.max(0)
   if np.any(c<lo-3.3) or np.any(c>hi+3.3):continue
   cross=[j for j in range(3) if j!=axis]
   # Confirm a real annular bore on at least one insertion side.
   valid=False
   for depth in [-4.,4.]:
    points=[]
    for radius in [2.,3.5]:
     for theta in np.linspace(0,2*np.pi,16,endpoint=False):
      v=c.copy();v[axis]+=depth;v[cross[0]]+=radius*np.cos(theta);v[cross[1]]+=radius*np.sin(theta);points.append(v)
    inside=tm.contains(points)
    if not inside[:16].any() and inside[16:].sum()>=6:valid=True
   if valid:cutters.append(cyl(3.25,c[axis]-1.1,c[axis]+1.1,axis,c))
  if cutters:
   s=solid(a)
   for cutter in cutters:s-=cutter
   arrays[part_index]=mesh(s)
 # Reindex after edits and persist exact world bounds.
 off=0
 for p,a in zip(parts,arrays):p.update(offset=off,vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()]);off+=a.size
