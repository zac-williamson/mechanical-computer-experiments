"""Shaft-scheduled complete bearing lands with obstacle-routed supporting webs.
No legacy transmission wall geometry is used. Stations and routing are audited.
"""
import numpy as np,json,heapq
from shapely.geometry import MultiPoint,Point,LineString
from shapely.ops import unary_union
from wall_pose import vertices,example_frames,joint,transform

def rebuild_bearings(g):
 traces=json.loads((g['BASE']/'Compact contact-resolved operation.json').read_text())['cases']
 frames=[case['frames'][i] for case in traces for i in np.linspace(0,len(case['frames'])-1,17,dtype=int)]
 P,A=g['P'],g['A'];cyl,box,emit=g['cyl'],g['box'],g['emit']
 # Continuous shaft assemblies; left/right pieces may be joined by clutch hubs.
 schedule=[
 ('M output',['master output left 3L','master output right 6L'],[-24,34]),
 ('Q output',['slave output left 4L','slave output right 7L'],[80,132]),
 ('Master worm',['master_gate worm drive 10L'],[-30,20]),
 ('Slave worm',['slave_gate left clutch stub 4L','slave_gate worm drive 10L'],[30.2,132]),
 ('Selected data',['WRITE selector output left','WRITE selector output right 7L'],[-102.1,-30]),
 ('D inverted',['WRITE D input axle 5L'],[-90,-72]),
 ('D input',['D external input 5L'],[-102.1,-72]),
 ('Master data stub',['Master gate data stub 3L'],[-50,-38]),
 ('POWER minus',['POWER distribution 12L'],[10,80]),
 ('POWER plus',['Shared reverse shaft left 10L','Shared reverse shaft right 10L'],[-24,80,132]),
 ('POWER input',['POWER local input 4L'],[60,80]),
 ('Feedback',['Q feedback shaft 32L'],[-102.1,34,144]),
 ('CLOCK worm',['Control CLK input axle'],[-126,-70]),
 ('CLOCK input',['Control CLK external axle'],[-144,-126]),
 ('WRITE worm',['Control WRITE input axle'],[-214,-154])]
 # Sampled sweeps span every inherited transition, including WRITE changes.
 scene=[]
 for p,a in zip(P,A):
  if p['module'] not in ['bit','control','coupler'] or p['kind']=='elastic':continue
  if p['kind']=='printed' and p.get('motion','fixed')=='fixed':
   # Protect existing working guide/actuator faces. Only rear structural
   # interfaces may become locating seats for the removable wall.
   clipped=g['solid'](a)^box([-300,-60,-400],[300,30.2,200])
   for component in clipped.decompose():
    if component.volume()<.001:continue
    mm=component.to_mesh64();vv=mm.vert_properties[:,:3][mm.tri_verts].reshape(-1,3)
    scene.append((dict(p,kind='working_clearance'),vv,[np.eye(4)],vv.min(0),vv.max(0)))
   continue
  seen=set();matrices=[];centre=(a.min(0)+a.max(0))/2;lo=a.min(0);hi=a.max(0)
  for f in frames:
   tf=transform(p,f)
   if p['kind']=='native':
    delta=tf[:3,:3]@centre+tf[:3,3]-centre;key=tuple(np.round(delta,5));matrix=np.eye(4);matrix[:3,3]=delta
   else:key=tuple(np.round(tf.ravel(),5));matrix=tf
   if key in seen:continue
   seen.add(key);matrices.append(matrix)
  lows=[];highs=[]
  for matrix in matrices:
   moved=a@matrix[:3,:3].T+matrix[:3,3];lows.append(moved.min(0));highs.append(moved.max(0))
  scene.append((p,a,matrices,np.min(lows,axis=0),np.max(highs,axis=0)))
 reports=[];planned=[];retention=[];plate_sections={}
 for name,names,targets in schedule:
  ids=[next(i for i,p in enumerate(P) if p['id']==n) for n in names]
  vs=np.concatenate([A[i] for i in ids]);axis=int(np.argmax(np.ptp(vs,axis=0)));cross=[1,2 if axis==0 else 0]
  c=(vs.min(0)+vs.max(0))/2;module=P[ids[0]]['module'];r=5.;half=1.2 if name=='Slave worm' else 1.6
  rear=41 if module=='bit' else 49.4
  anchors=[-28,40,56] if module=='bit' else [-164,-108]
  def cross_obstacles(t,half):
   shapes=[];shape_names=[]
   for p,base,matrices,lo,hi in scene:
    if p['module'] not in [module,'coupler'] or p['id'] in names:continue
    if lo[axis]>=t+half+.15 or hi[axis]<=t-half-.15:continue
    # Stream poses: retaining all transformed triangle arrays caused excessive RAM use.
    projected=[]
    for matrix in matrices:
     a=base@matrix[:3,:3].T+matrix[:3,3]
     tri=a.reshape(-1,3,3);ok=(tri[:,:,axis].min(1)<t+half+.15)&(tri[:,:,axis].max(1)>t-half-.15)
     tri=tri[ok]
     if not len(tri):continue
     flat=tri.reshape(-1,3);inside=flat[(flat[:,axis]>=t-half-.15)&(flat[:,axis]<=t+half+.15)]
     edges=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]);dd=edges[:,1,axis]-edges[:,0,axis];edges=edges[abs(dd)>1e-9];dd=edges[:,1,axis]-edges[:,0,axis]
     clips=[inside]
     for plane in [t-half-.15,t+half+.15]:
      frac=(plane-edges[:,0,axis])/dd;valid=(frac>=0)&(frac<=1)
      clips.append(edges[valid,0]+frac[valid,None]*(edges[valid,1]-edges[valid,0]))
     projected.append(np.concatenate(clips)[:,cross])
    pts=np.concatenate(projected) if projected else np.empty((0,2))
    if not len(pts):continue
    shape=MultiPoint(np.unique(np.round(pts,5),axis=0)).convex_hull
    if p['kind']=='native':
     # Bound rotation with a circular radial envelope in the support plane.
     aa=int(np.argmax(hi-lo)) if p.get('lego_part') in ['3705','3706','3707','3708','3737','4519','44294','50450','60485','59443'] else p.get('axis',int(np.argmax(abs(joint(p,frames[0])[1]))))
     if aa==axis:
      cc=(lo+hi)/2;rr=float(np.linalg.norm(pts-cc[cross],axis=1).max());shape=Point(cc[cross]).buffer(rr,resolution=12)
    if p['kind']=='native' and aa!=axis:shape=shape.buffer(.4,resolution=3)
    shapes.append(shape);shape_names.append(p['id'])
   return unary_union(shapes),shapes,shape_names
  used=[]
  for target in targets:
   found=None
   for t in sorted(set([target]+[round(target+d,2) for d in np.arange(-12,12.1,2)]),key=lambda x:abs(x-target)):
    if any(abs(t-q)<6 for q in used):continue
    if not any(A[i][:,axis].min()+half+.2<=t<=A[i][:,axis].max()-half-.2 for i in ids):continue
    obstacle,shapes,shape_names=cross_obstacles(t,half)
    ring=Point(c[cross]).buffer(r,resolution=16).difference(Point(c[cross]).buffer(2.65,resolution=16))
    if ring.intersection(obstacle).area>1e-5:
     if t==target:print('REJECT RING',name,t,[(n,round(ring.intersection(sh).area,2)) for n,sh in zip(shape_names,shapes) if ring.intersection(sh).area>1e-5],flush=True)
     continue
    expanded=obstacle.buffer(2.25,resolution=3)
    start=tuple(c[cross]);goal=(rear,min(anchors,key=lambda z:abs(z-start[1])))
    # A* on a 2 mm lattice, then visibility-shortcut the clear path.
    forbidden=expanded.buffer(-.015)
    goals=[(rear,z) for z in sorted(anchors,key=lambda z:abs(z-start[1])) if not forbidden.contains(Point(rear,z))]
    if goals:goal=goals[0]
    if forbidden.contains(Point(start)) or not goals:
     if t==target:print('REJECT END',name,t,flush=True)
     continue
    step=2.;origin=np.array(start);dist={(0,0):0.};prev={};queue=[(np.linalg.norm(origin-goal),0.,(0,0))];finish=None
    def pos(k):return origin+step*np.array(k)
    while queue:
     _,cost,k=heapq.heappop(queue)
     if cost!=dist.get(k):continue
     here=pos(k)
     if not LineString([here,goal]).intersects(forbidden):finish=k;break
     for du,dv in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
      kk=(k[0]+du,k[1]+dv);xy=pos(kk)
      if not (-25<=xy[0]<=rear+1 and (-48<=xy[1]<=66 if module=='bit' else -172<=xy[1]<=-82)):continue
      d=cost+step*np.hypot(du,dv)
      if d>=dist.get(kk,float('inf')):continue
      if LineString([here,xy]).intersects(forbidden):continue
      dist[kk]=d;prev[kk]=k;heapq.heappush(queue,(d+np.linalg.norm(xy-goal),d,kk))
    if finish is None:continue
    keys=[finish]
    while keys[-1]!=(0,0):keys.append(prev[keys[-1]])
    raw=[pos(k).tolist() for k in keys[::-1]]+[list(goal)];path=[raw[0]];j=0
    while j<len(raw)-1:
     jj=next(q for q in range(len(raw)-1,j,-1) if not LineString([raw[j],raw[q]]).intersects(forbidden))
     path.append(raw[jj]);j=jj
    def disc(xy,radius):
     cc=c.copy();cc[axis]=t;cc[cross]=xy;return cyl(radius,t-half,t+half,axis,cc)
    s=disc(start,r)
    for a,b in zip(path,path[1:]):s+=(disc(a,2)+disc(b,2)).hull()
    s-=cyl(2.65,t-half-.1,t+half+.1,axis,c)
    found=(t,s,path);break
   if found is None:
    reports.append(dict(shaft=name,target=target,status='NO_CLEAR_COMPLETE_BEARING'));print('NO BEARING',name,target,flush=True);continue
   t,s,path=found;used.append(t);planned.append((module,t,s,name))
   # A broad plate carries bending loads; the routed 4 mm strip is only a
   # clearance fallback at constrained necks, never the default entire wall.
   key=(module,t)
   section=plate_sections.setdefault(key,dict(axis=axis,half=half,profiles=[],obstacles=[],bores=[]))
   section['half']=min(section['half'],half)
   foot=path[-1]
   profile=unary_union([Point(path[0]).buffer(r,resolution=16),
      LineString([(foot[0],foot[1]-6),(foot[0],foot[1]+6)]).buffer(2,resolution=8)]).convex_hull
   section['profiles'].append(profile)
   section['obstacles'].append(obstacle.buffer(.35,resolution=6))
   section['bores'].append(Point(c[cross]).buffer(2.7,resolution=16))
   reports.append(dict(shaft=name,axle_parts=names,module=module,axis='XYZ'[axis],station_mm=t,land_mm=2*half,radial_wall_mm=2.35,path_depth_and_cross_mm=path,status='complete bearing generated'))
   print('BEARING',name,t,flush=True)
  # A shaft can be retained at one bearing or at opposite outer bearings.
  def stop_candidate(t,side):
   q=t+side*(half+2.2);face=t+side*(half+.2)
   for p,a in zip(P,A):
    if p['module']!=module or p['kind']!='native' or not ('retainer' in p['id'].lower() or p['id'].endswith((' L069',' L105'))):continue
    lo,hi=a.min(0),a.max(0);cc=(lo+hi)/2
    if np.linalg.norm(cc[cross]-c[cross])>.02:continue
    stopface=hi[axis] if side<0 else lo[axis]
    if abs(stopface-face)<.06:return dict(bearing=t,side=side,existing=p['id'],face_mm=float(stopface))
   if not any(A[i][:,axis].min()<=q-1.9 and A[i][:,axis].max()>=q+1.9 for i in ids):return None
   if Point(c[cross]).buffer(3.65,resolution=16).intersection(cross_obstacles(q,1.9)[0]).area>1e-5:return None
   return dict(bearing=t,side=side,centre=q)
  left=[x for t in used if (x:=stop_candidate(t,-1)) is not None]
  right=[x for t in used if (x:=stop_candidate(t,1)) is not None]
  if left and right:
   pair=min(((a,b) for a in left for b in right),key=lambda ab:abs(ab[0]['bearing']-ab[1]['bearing']))
   source=P[ids[0]]
   for stop in pair:
    if 'existing' in stop:continue
    cc=c.copy();cc[axis]=stop['centre']
    collar=g['native'](name+' axial stop '+str(round(stop['centre'],3)),'4265c',cc,axis=axis,module=module)
    for field in ['baseline_id','placement_shift','assembly_rotation','assembly_translation','drive','phase_deg']:
     if field in source:collar[field]=source[field]
    if 'assembly_rotation' in source and 'baseline_id' not in source:
     rr=np.array(source['assembly_rotation']);tt=np.array(source['assembly_translation'])
     collar['centre']=(rr.T@(cc-tt)).tolist()
   retention.append(dict(shaft=name,stops=pair,axial_clearance_each_side_mm=.2,status='opposed axial stops assigned'))
  else:retention.append(dict(shaft=name,status='NO_CLEAR_OPPOSED_STOPS',left_candidates=left,right_candidates=right));print('NO STOPS',name,flush=True)
 assert all(b['status']=='complete bearing generated' for b in reports), 'Missing bearing: inspect station search'
 assert all(r['status']=='opposed axial stops assigned' for r in retention), 'Missing axial retention'
 # Merge webs at shared stations; each station is a deliberately drilled plate.
 groups={}
 for module,t,s,name in planned:
  key=(module,t)
  if key not in groups:groups[key]=[s,[name]]
  else:groups[key][0]+=s;groups[key][1].append(name)
 plate_audit=[]
 for (module,t),(s,names) in groups.items():
  section=plate_sections[(module,t)]
  profiles=section['profiles']
  # Adjacent bearing heads at a common station form one drilled plate.
  profile=unary_union(profiles).convex_hull
  clear=profile.difference(unary_union(section['obstacles']+section['bores'])).simplify(.015,preserve_topology=True)
  polygons=list(clear.geoms) if clear.geom_type=='MultiPolygon' else [clear]
  added=g['m'].Manifold()
  for poly in polygons:
   if poly.geom_type!='Polygon' or poly.area<.01:continue
   contours=[np.asarray(poly.exterior.coords)[:-1]]+[np.asarray(h.coords)[:-1] for h in poly.interiors]
   plate=g['m'].CrossSection(contours,g['m'].FillRule.EvenOdd).extrude(2*section['half']-.1)
   if section['axis']==0:
    plate=plate.transform([[0,0,1,t-section['half']+.05],[1,0,0,0],[0,1,0,0]])
   else:
    plate=plate.transform([[0,1,0,0],[1,0,0,0],[0,0,1,t-section['half']+.05]])
   # Only retain material attached to the original support network.
   for component in plate.decompose():
    if (component^s).volume()>.01:added+=component
  extra=(added-s).volume();s+=added
  plate_audit.append(dict(module=module,station_mm=t,shafts=names,additional_bracing_mm3=extra,
    attachment='integral with coordinated chassis',rear_foot_width_mm=16))
  emit(module+' bearing web '+str(t)+' '+', '.join(names),s,module=module,motion='fixed',color=(.3,.49,.47))
 # Manufacture bearing walls independently, with their axle holes vertical
 # on the print bed. Mounting pads face the frame across a 0.4 mm gap.
 attachments=[]
 for module in ['bit','control']:
  interface=40.5 if module=='bit' else 48.5
  axis=0 if module=='bit' else 2
  cross=2 if axis==0 else 0
  wallids=[i for i,p in enumerate(P) if p['id'].startswith(module+' bearing web ')]
  wall=g['m'].Manifold();candidates=[]
  for i in wallids:
   p=P[i];a=A[i];ss=g['solid'](a)
   t=float(p['id'].split('bearing web ')[1].split(' ')[0])
   entries=[b for b in reports if b['module']==module and b['station_mm']==t]
   feet=[-30 if t < -65 else -28] if module=='bit' else [-164]
   # A rear heel meets the actual routed wall before the frame interface;
   # do not clip off a route and leave its mounting pads disconnected.
   lo=a.min(0).copy();hi=a.max(0).copy();lo[1]=interface-4.2;hi[1]=interface-.2
   lo[cross]=min(lo[cross],min(feet)-9.8);hi[cross]=max(hi[cross],max(feet)+9.8)
   heel=box(lo,hi)
   if module=='bit':
    heel-=g['link']((-108,-16),(-100,-16),3.95,13.8,41.2)
    heel-=g['link']((-106,21),(-86,21),3.95,22,41.2)
   for heel_piece in heel.decompose():
    if (heel_piece^ss).volume()>.001:ss+=heel_piece
   for foot in feet:
    for z in [foot-6,foot+6]:
     c=np.zeros(3);c[axis]=float(a[:,axis].min())+3.8;c[1]=interface;c[cross]=z
     lo=c-np.array([3.8,7.8,3.8]);hi=c+np.array([3.8,-.2,3.8])
     pad=box(lo,hi)
     # Bridge the two pad roots with a broad heel in the bearing plane.
     cc=c.copy();cc[cross]=foot
     lo2=cc-np.array([3.8,7.8,3.8]);hi2=cc+np.array([3.8,-.2,3.8])
     ss+=(pad+box(lo2,hi2)).hull()
     candidates.append(c)
   wall+=ss^box([-300,-60,-400],[300,interface-.2,200])
  frameids=[i for i,p in enumerate(P) if p['module']==module and p['kind']=='printed' and p.get('motion','fixed')=='fixed' and A[i][:,1].max()>35 and i not in wallids]
  frame=sum((g['solid'](A[i]) for i in frameids),g['m'].Manifold())
  for i in sorted(wallids+frameids,reverse=True):P.pop(i);A.pop(i)
  walls=list(wall.decompose())
  for j,ss in enumerate(walls):
   valid=[]
   for c in candidates:
    probe=box(c+[-1,-6,-1],c+[1,-2,1])
    if (probe^ss).volume()>15.9 and all(np.linalg.norm(c-d)>1 for d in valid):valid.append(c)
   pairs=[(a,b) for k,a in enumerate(valid) for b in valid[k+1:] if np.linalg.norm(a-b)>=10]
   assert pairs,('No separated mount pair',module,j,ss.bounding_box())
   pair=max(pairs,key=lambda ab:np.linalg.norm(ab[0]-ab[1]))
   pins=[]
   for k,c in enumerate(pair):
    # Matching frame sockets share the existing rear rail, not a lone peg.
    frame+=box(c+[-3.8,.2,-3.8],c+[3.8,7.8,3.8])
    near=min([-28,40,56] if module=='bit' else [-164,-108],key=lambda q:abs(q-c[cross]))
    lo=c+[-3.8,.2,-3.8];hi=c+[3.8,6,3.8]
    lo[cross]=min(c[cross],near)-3.8;hi[cross]=max(c[cross],near)+3.8
    frame+=box(lo,hi)
    bore=cyl(2.5,interface-8,interface+8,1,c)
    ss-=bore;frame-=bore
    name=module+' bearing wall '+str(j)+' mounting pin '+str(k+1)
    g['native'](name,'2780',c,axis=1,module=module)
    pins.append(dict(part=name,centre_mm=c.tolist()))
   # A shallow locating seat provides lateral registration. Its clearance is
   # modelled explicitly; bearing wall and chassis remain separate solids.
   seat=ss
   for d in [[.2,0,0],[-.2,0,0],[0,.2,0],[0,-.2,0],[0,0,.2],[0,0,-.2]]:seat+=ss.translate(d)
   frame-=seat
   name=module+' removable bearing wall '+str(j)
   emit(name,ss,module=module,motion='fixed',color=(.3,.49,.47))
   attachments.append(dict(part=name,module=module,pins=pins,pin_spacing_mm=float(np.linalg.norm(pair[0]-pair[1])),bearing_axis='XYZ'[axis],print_bed_normal='XYZ'[axis],frame_interface_depth_mm=interface))
  for mount in attachments:
   if mount['module']==module:
    for pin in mount['pins']:frame-=cyl(2.5,interface-8,interface+8,1,pin['centre_mm'])
  if module=='bit':
   # Split LAST: no socket, heel or later union may bridge this interface.
   for zz in [-28,56]:
    frame-=cyl(2.5,20.4,36.6,0,[0,44.5,zz])+cyl(3.3,28.1,28.9,0,[0,44.5,zz])
   frame-=box([28.3,35,-400],[28.7,100,200])
   assert len([s for s in frame.decompose() if s.volume()>.01])==2,[(x.volume(),x.bounding_box()) for x in frame.decompose()]
  for j,ss in enumerate(frame.decompose()):
   if ss.volume()>.01:emit(module+' coordinated chassis '+str(j),ss,module=module,motion='fixed',color=(.3,.49,.47))
 (g['OUT']/'Bearing schedule.json').write_text(json.dumps(dict(scope='Complete ring and routed web candidates; subsequent three-dimensional collision, connection and retention checks required',bearings=reports,retention=retention,braced_plates=plate_audit,removable_walls=attachments),indent=2))
