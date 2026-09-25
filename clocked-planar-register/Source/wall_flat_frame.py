"""Separate working fixtures from flat-backed frames; use paired LEGO friction-pin attachments."""
import numpy as np
import trimesh
import manifold3d as m
import json
from shapely.geometry import Polygon
from shapely.ops import unary_union,nearest_points

def project_back(s,back):
 """Extend every rear-facing surface to a common bed plane."""
 q=s.to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True)
 solids=[s]
 for tri in t.triangles[t.face_normals[:,1]>1e-6]:
  if np.max(back-tri[:,1])<1e-6:continue
  end=tri.copy();end[:,1]=back
  solids.append(m.Manifold.hull_points(np.vstack([tri,end])))
 return m.Manifold.batch_boolean(solids,m.OpType.Add)


def native_envelope(p,a,joint,f):
 """Same axial-step envelope used by the independent rotating-part screen."""
 sh,ax,cen,ang=joint(p,f);axis=int(np.argmax(abs(ax)))
 if p.get('lego_part')=='2780':axis=int(np.argmax(np.ptp(a,axis=0)))
 elif p.get('axis') is not None:axis=p['axis']
 elif p.get('motion')=='gear' or 'stop-axle' in p['id'] or 'retainer' in p['id']:axis=1
 cross=[j for j in range(3) if j!=axis];centre=(a.min(0)+a.max(0))/2
 tri=a.reshape(-1,3,3);edges=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]);dx=edges[:,1,axis]-edges[:,0,axis]
 edges=edges[abs(dx)>1e-9];dx=edges[:,1,axis]-edges[:,0,axis]
 xx=np.unique(np.round(a[:,axis],4));eps=1e-5
 grid=np.unique(np.clip(np.r_[xx-eps,xx+eps,(xx[:-1]+xx[1:])/2],a[:,axis].min()+1e-7,a[:,axis].max()-1e-7))
 radii=[];positions=[]
 for position in grid:
  frac=(position-edges[:,0,axis])/dx;ok=(frac>=0)&(frac<=1)
  if not ok.any():continue
  pts=edges[ok,0]+frac[ok,None]*(edges[ok,1]-edges[ok,0])
  radii.append(np.linalg.norm(pts[:,cross]-centre[cross],axis=1).max());positions.append(position)
 profile=np.vstack([[0,positions[0]],np.stack([radii,positions],axis=1),[0,positions[-1]]])
 mesh=trimesh.creation.revolve(profile,sections=64)
 mesh.apply_transform(trimesh.geometry.align_vectors([0,0,1],np.eye(3)[axis]));delta=centre.copy();delta[axis]=0;mesh.apply_translation(delta)
 return m.Manifold(m.Mesh64(mesh.vertices.astype(float),mesh.faces.astype(np.uint64)))

def rebuild_flat_frames(g):
 P,A=g['P'],g['A'];box,cyl=g['box'],g['cyl'];records=[]
 # Work on the existing split, without rerouting gears, bearings or controls.
 ids=[i for i,p in enumerate(P) if 'coordinated chassis' in p['id']]
 saved=[(P[i].copy(),g['solid'](A[i])) for i in ids]
 obstacles=[]
 from wall_pose import vertices,transform,joint
 cases=json.loads((g['BASE']/'Compact contact-resolved operation.json').read_text())['cases']
 frames=[case['frames'][i] for case in cases for i in np.linspace(0,len(case['frames'])-1,17,dtype=int)]
 for p,a in zip(P,A):
  if p['kind']=='elastic' or 'coordinated chassis' in p['id']:continue
  if p['kind']=='native':
   a=native_envelope(p,a,joint,frames[0]).to_mesh64()
   a=a.vert_properties[:,:3][a.tri_verts].reshape(-1,3)
  centre=(a.min(0)+a.max(0))/2;shape=g['solid'](a)
  seen=set()
  for f in frames:
   tf=transform(p,f)
   if p['kind']=='native':
    delta=tf[:3,:3]@centre+tf[:3,3]-centre;key=tuple(np.round(delta,5))
   else:key=tuple(np.round(tf.ravel(),5))
   if key in seen:continue
   seen.add(key)
   ss=shape.translate(delta) if p['kind']=='native' else shape.transform(tf[:3])
   lo,hi=np.array(ss.bounding_box()).reshape(2,3)
   obstacles.append((p['id'],lo,hi,ss))
 print('FRAME ROUTING OBSTACLES',len(obstacles),flush=True)
 for i in reversed(ids):P.pop(i);A.pop(i)
 protected=None;reserved_ignore=set();foot_limit=None;cache_len=-1;cache_lo=None;cache_hi=None
 def clear(s,ignore=()):
  nonlocal cache_len,cache_lo,cache_hi
  if protected is not None and (s^protected).volume()>.005:return False
  bb=np.array(s.bounding_box()).reshape(2,3)
  if foot_limit is not None and (np.any(bb[0,[0,2]]<foot_limit[0,[0,2]]-1e-6) or np.any(bb[1,[0,2]]>foot_limit[1,[0,2]]+1e-6)):return False
  if cache_len!=len(obstacles):
   cache_lo=np.array([x[1] for x in obstacles]);cache_hi=np.array([x[2] for x in obstacles]);cache_len=len(obstacles)
  indices=np.flatnonzero(np.all(np.minimum(bb[1],cache_hi)-np.maximum(bb[0],cache_lo)>1e-5,axis=1))
  for ix in indices:
   name,lo,hi,o=obstacles[ix]
   if name in ignore or name in reserved_ignore:continue
   if (s^o).volume()>.005:return False
  return True
 for p,original in saved:
  foot_limit=np.array(original.bounding_box()).reshape(2,3)
  module=p['module'];interface=38.0 if module=='bit' else 48.3;back=48.5 if module=='bit' else 56.3
  if module=='control':
   reserved_stem=box([-113,17.8,-77],[-110,interface-.2,-73.8])
   for xx in [-110,-102]:
    cc=np.array([xx,interface,-78.])
    reserved_stem+=box([xx-3,back-16.2,-81],[xx+3,interface-.2,-75])+g['link']([xx,-78],[-111,-76],2.3,interface-2.8,interface-.2)
   obstacles.append(('Reserved upper guide root',*np.array(reserved_stem.bounding_box()).reshape(2,3),reserved_stem))
  front=original^box([-300,-60,-400],[300,interface-.2,200])
  base=original^box([-300,interface+.2,-400],[300,back,200])
  components=[s for s in front.decompose() if s.volume()>.01]
  fixtures=[]
  for s in components:
   bb=np.array(s.bounding_box()).reshape(2,3)
   if bb[0,1]>interface-8:
    base+=project_back(s,back)
   else:fixtures.append(s)
  if module=='bit':
   for zz in [-28,56]:
    boss=box([20.5,40.5,zz-4],[36.5,back,zz+4])
    # Square-backed joining bosses remove the former rounded bed contact.
    if p['id'].endswith(' 0'):boss ^= box([-300,-60,-400],[28.3,100,200])
    else:boss ^= box([28.7,-60,-400],[300,100,200])
    base+=boss
  base=project_back(base,back)
  # Share a rear mounting heel when neighbouring guide roots can connect
  # behind their moving parts. Avoid one-pin fragments and duplicated feet.
  changed=True
  while changed:
   changed=False
   for i,sa in enumerate(fixtures):
    if changed:break
    for k in range(i+1,len(fixtures)):
     sb=fixtures[k]
     ba=np.array(sa.bounding_box()).reshape(2,3);bb=np.array(sb.bounding_box()).reshape(2,3)
     gap=np.maximum(0,np.maximum(ba[0,[0,2]]-bb[1,[0,2]],bb[0,[0,2]]-ba[1,[0,2]]))
     if np.linalg.norm(gap)>(30 if module=='control' else 12):continue
     slab=box([-300,interface-(1.8 if module=='control' else 2.8),-400],[300,interface-.2,200])
     ra=sa^slab;rb=sb^slab
     if ra.is_empty() or rb.is_empty():continue
     heel=(ra+rb).hull()
     if not clear(heel) and module=='control':
      # Connect the actual nearest rear roots, not centres of broad bounding
      # boxes that may lie across a transmission wall or amplifier pivot.
      profiles=[]
      for root_piece in [ra,rb]:
       mm=root_piece.to_mesh64();tri=mm.vert_properties[:,:3][mm.tri_verts]
       poly=unary_union([Polygon(t[:,[0,2]]) for t in tri if Polygon(t[:,[0,2]]).area>1e-8])
       inset=poly.buffer(-.5);profiles.append(inset if not inset.is_empty else poly)
      aa,bb2=nearest_points(*profiles)
      candidate=g['link'](np.array(aa.coords[0]),np.array(bb2.coords[0]),2.3,interface-1.8,interface-.2)
      bounds=np.array([ra.bounding_box(),rb.bounding_box()]).reshape(2,2,3)
      candidate ^= box(bounds[:,0].min(0),bounds[:,1].max(0))
      if clear(candidate) and (candidate^sa).volume()>1 and (candidate^sb).volume()>1:heel=candidate
     if not clear(heel):
      heel=None
      for ax in [0,2]:
       other=2 if ax==0 else 0
       for side in [-1,1]:
        route=(min(ba[0,ax],bb[0,ax])-3 if side<0 else max(ba[1,ax],bb[1,ax])+3)
        ca=(ba[0]+ba[1])/2;cb=(bb[0]+bb[1])/2
        ca[ax]=ba[0 if side<0 else 1,ax]-side*1.8;cb[ax]=bb[0 if side<0 else 1,ax]-side*1.8
        legs=[]
        for cc in [ca,cb]:
         lo=cc.copy();hi=cc.copy();lo[ax]=min(route,cc[ax])-1.8;hi[ax]=max(route,cc[ax])+1.8
         lo[other]-=1.8;hi[other]+=1.8;lo[1]=interface-(1.8 if module=='control' else 2.8);hi[1]=interface-.2
         legs.append(box(lo,hi))
        lo=ca.copy();hi=cb.copy();lo[ax]=route-1.8;hi[ax]=route+1.8
        lo[other]=min(ca[other],cb[other])-1.8;hi[other]=max(ca[other],cb[other])+1.8;lo[1]=interface-(1.8 if module=='control' else 2.8);hi[1]=interface-.2
        candidate=legs[0]+legs[1]+box(lo,hi)
        if clear(candidate) and (candidate^sa).volume()>1 and (candidate^sb).volume()>1:
         heel=candidate;break
       if heel is not None:break
      if heel is None:continue
     fixtures[i]=sa+sb+heel;fixtures.pop(k);changed=True;break
  if module=='control':
   # Route the rear CLOCK cheek around the existing transmission-wall pin.
   # The bridge is behind the amplifier and its pivot hardware, below 48.1 Y.
   bridge=g['link']([-147.3,-122],[-147.3,-114],2.3,46.5,48.08)+g['link']([-147.3,-114],[-163.5,-114],2.3,46.5,48.08)
   assert clear(bridge), 'Controller rear-cheek bridge obstruction'
   touching=[k for k,ss in enumerate(fixtures) if (bridge^ss).volume()>.001]
   assert touching
   combined=bridge
   for k in touching:combined+=fixtures[k]
   for k in sorted(touching,reverse=True):fixtures.pop(k)
   fixtures.insert(min(touching),combined)
  # The WRITE pivot's narrow separate foot cannot fit two 2L pin sockets.
  # Share a braced heel with the adjacent lower guide, keeping two remote pins.
  if p['id']=='bit coordinated chassis 0':
   pivot_probe=cyl(3.5,35.3,37.7,1,[-104,0,8])-cyl(2.8,35.2,37.8,1,[-104,0,8])
   lower_probe=box([-109,35.3,-30],[-105,37.7,-25])
   ip=max(range(len(fixtures)),key=lambda k:(fixtures[k]^pivot_probe).volume())
   il=max(range(len(fixtures)),key=lambda k:(fixtures[k]^lower_probe).volume())
   assert ip!=il
   bridge=None
   for route in [-116.,-118.,-120.,-122.]:
    candidate=g['link']([route,-27],[route,6],2.4,35.2,37.72)+g['link']([route,6],[-109.8,6],2.4,35.2,37.72)+g['link']([route,-27],[-108,-27],2.4,35.2,37.72)
    if clear(candidate):bridge=candidate;break
   if bridge is None:raise ValueError('No clear braced WRITE pivot route')
   touching=[k for k,ss in enumerate(fixtures) if (bridge^ss).volume()>.001]
   assert ip in touching and il in touching
   combined=bridge
   for k in touching:combined+=fixtures[k]
   for k in sorted(touching,reverse=True):fixtures.pop(k)
   fixtures.insert(min(touching),combined)
  for j,s in enumerate(fixtures):
   protected=front-s;reserved_ignore=set()
   bb=np.array(s.bounding_box()).reshape(2,3)
   if module=='control' and bb[0,2]<-72<bb[1,2] and bb[1,0]>-111 and bb[0,1]<10:
    # The upper rod guide mounts on its right flank, clear of the nearby
    # removable bearing wall; its rod-contact faces stay unchanged.
    reserved_ignore={'Reserved upper guide root'}
    stem=box([-113,17.8,-77],[-110,interface-.2,-73.8])
    if not clear(stem):
     print('STEM OBSTACLES',[(n,(stem^o).volume()) for n,lo,hi,o in obstacles if (stem^o).volume()>.005], 'reserved',(stem^protected).volume(),flush=True)
     raise ValueError('Upper guide right-hand root obstructed')
    s+=stem
   # Prefer the original root footprint. Search only its immediate vicinity.
   root=s^box([-300,interface-4,-400],[300,interface,200])
   rb=np.array(root.bounding_box()).reshape(2,3)
   candidates=[]
   # Existing bored frame-pin positions have first preference.
   known=[np.array([xx,interface,-78.]) for xx in [-110,-102]] if reserved_ignore else []
   for old in g['OLD']:
    if old.get('lego_part')!='2780' or not ('frame pin' in old['id'] or 'track mount' in old['id']):continue
    c=np.array(old['bounds']).mean(0)
    if module=='control':
     bank='clock' if 'clock' in old['id'] else 'write' if 'write' in old['id'] else None
     if not bank:continue
     R=np.array([[0,0,1],[0,1,0],[-1,0,0]])
     c=R@c+np.array([-100,-4,-64] if bank=='clock' else [-104,-12,-260])
    c[1]=interface
    known.append(c)
   grid=[np.array([x,interface,z]) for x in np.arange(np.floor(rb[0,0])-18,np.ceil(rb[1,0])+19,2) for z in np.arange(np.floor(rb[0,2])-18,np.ceil(rb[1,2])+19,2)]
   for c in known+grid:
    if np.any(c[[0,2]]<rb[0,[0,2]]-18) or np.any(c[[0,2]]>rb[1,[0,2]]+18):continue
    pad=box(c+[-3.8,-7.8,-3.8],c+[3.8,-.2,3.8]);bore=cyl(2.5,interface-8.1,back+.1,1,c)
    original_bounds=np.array(original.bounding_box()).reshape(2,3)
    if np.any(c[[0,2]]-3.8<original_bounds[0,[0,2]]) or np.any(c[[0,2]]+3.8>original_bounds[1,[0,2]]):continue
    if (pad^s).volume()<15:
     target=np.clip(c[[0,2]],rb[0,[0,2]]+1,rb[1,[0,2]]-1)
     neck=g['link'](c[[0,2]],target,2.3,interface-2.8,interface-.2)
     if (neck^s).volume()<2:continue
     pad+=neck
    # Reserve the full friction-pin envelope, including its centre collar.
    envelope=cyl(2.6,interface-8,interface+8,1,c)+cyl(3.3,interface-.4,interface+.4,1,c)
    if not clear(pad+envelope):continue
    if any(np.linalg.norm(c-d)<.1 for d,_ in candidates):continue
    candidates.append((c,pad))
   print('FIXTURE',p['id'],j,'candidates',len(candidates),flush=True)
   pairs=[(a,b) for k,a in enumerate(candidates) for b in candidates[k+1:] if np.linalg.norm(a[0]-b[0])>=6]
   if not pairs:raise ValueError(('No paired fixture mount',p['id'],j,bb.tolist(),[(c.tolist()) for c,_ in candidates]))
   a,b=max(pairs,key=lambda ab:np.linalg.norm(ab[0][0]-ab[1][0]))
   mounts=[a,b]
   if module=='control' and bb[1,2]-bb[0,2]>40:
    while len(mounts)<4:
     remaining=[q for q in candidates if min(np.linalg.norm(q[0]-r[0]) for r in mounts)>=10]
     if not remaining:raise ValueError('Controller carrier needs four spaced pins')
     mounts.append(max(remaining,key=lambda q:min(np.linalg.norm(q[0]-r[0]) for r in mounts)))
   pins=[]
   for k,(c,pad) in enumerate(mounts):
    bore=cyl(2.5,interface-8.1,back+.1,1,c)+cyl(3.3,interface-.4,interface+.4,1,c)
    s=(s+pad)-bore
    foot=box(c+[-3.8,.2,-3.8],[c[0]+3.8,back,c[2]+3.8])
    foot+=project_back(pad,back)^box([-300,interface+.2,-400],[300,back,200])
    base=(base+foot)-bore
    name=p['id'].replace('coordinated chassis','frame')+' fixture '+str(j)+' pin '+str(k+1)
    g['native'](name,'2780',c,axis=1,module=module)
    pins.append(dict(part=name,centre_mm=c.tolist()))
   for pin in pins:
    s-=cyl(2.5,interface-8.1,back+.1,1,pin['centre_mm'])+cyl(3.3,interface-.4,interface+.4,1,pin['centre_mm'])
   name=p['id'].replace('coordinated chassis','frame')+' removable fixture '+str(j)
   g['emit'](name,s,module=module,motion='fixed',color=(.42,.65,.60))
   obstacles.append((name,*np.array(s.bounding_box()).reshape(2,3),s))
   records.append(dict(part=name,frame=p['id'],fasteners=pins,fastener_spacing_mm=float(np.linalg.norm(a[0]-b[0])),interface_y_mm=interface))
  # Join isolated socket feet to the rear lattice with bed-level ribs.
  def rear_shape(ss):
   mm=ss.to_mesh64();tt=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True)
   tri=tt.triangles[np.abs(tt.triangles[:,:,1]-back).max(1)<.001]
   return unary_union([Polygon(q[:,[0,2]]) for q in tri if Polygon(q[:,[0,2]]).area>1e-8])
  pieces=sorted([q for q in base.decompose() if q.volume()>.001],key=lambda q:-q.volume())
  joined=pieces.pop(0)
  while pieces:
   shape=rear_shape(joined)
   k=min(range(len(pieces)),key=lambda k:shape.distance(rear_shape(pieces[k])))
   piece=pieces.pop(k);a2,b2=nearest_points(shape,rear_shape(piece))
   rib=g['link'](np.array(a2.coords[0]),np.array(b2.coords[0]),3,back-3,back)
   ob=np.array(original.bounding_box()).reshape(2,3)
   rib ^= box([ob[0,0],back-3,ob[0,2]],[ob[1,0],back,ob[1,2]])
   joined+=piece+rib
  base=joined
  # Reopen the original vertical attachment passages through the added ribs.
  for q,a in zip(P,A):
   if q['module']==module and q.get('lego_part')=='2780' and q.get('axis')==1 and a[:,1].max()>interface+.2:
    c=(a.min(0)+a.max(0))/2
    base-=cyl(2.5,interface-.1,back+.1,1,c)
  protected=None;reserved_ignore=set()
  # Re-cut pin sockets after combining mounting feet.
  for record in records:
   if record['frame']!=p['id']:continue
   for fastener in record['fasteners']:
    c=np.array(fastener['centre_mm'])
    base-=cyl(2.5,interface-.1,back+.1,1,c)+cyl(3.3,interface-.4,interface+.4,1,c)
  # Preserve the split pin's round seating sides, with a printable 45-degree roof.
  if module=='bit':
   for z in [-28,56]:
    for radius,lo,hi in [(2.5,20.4,36.6),(3.3,28.1,28.9)]:
     # Assembly -Y is printer up. Tangents at 45 degrees meet above the circle.
     c=np.array([0,44.5,z]);r=radius
     pts=np.array([[lo,44.5-r/np.sqrt(2),z-r/np.sqrt(2)],[lo,44.5-r*np.sqrt(2),z],[lo,44.5-r/np.sqrt(2),z+r/np.sqrt(2)]])
     roof=m.Manifold.hull_points(np.vstack([pts,pts+np.array([hi-lo,0,0])]))
     base-=cyl(r,lo,hi,0,c)+roof
   # Both side faces remain independent even after new mount pads.
   if p['id'].endswith(' 0'):base=base^box([-300,-60,-400],[28.3,100,200])
   else:base=base^box([28.7,-60,-400],[300,100,200])
  # Front-opening seats around removable walls cannot create a print roof.
  for q,a in zip(P,A):
   if q['module']!=module or 'removable bearing wall' not in q['id']:continue
   contact=base^g['solid'](a)
   for contact_piece in contact.decompose():
    if contact_piece.volume()<.0001:continue
    lo,hi=np.array(contact_piece.bounding_box()).reshape(2,3)
    base-=box([lo[0]-.2,-60,lo[2]-.2],[hi[0]+.2,hi[1]+.2,hi[2]+.2])
  g['emit'](p['id'],base,module=module,motion='fixed',color=(.25,.43,.41))
 (g['OUT']/'Frame fixture schedule.json').write_text(json.dumps(dict(fixtures=records,print_direction='Rear face on bed; assembly -Y points upwards'),indent=2))

def reopen_control_pivots(g):
 """Keep the real bit-control axle and rear bush passages open through ribs."""
 for i,p in enumerate(g['P']):
  if not p['id'].startswith('bit coordinated chassis '):continue
  s=g['solid'](g['A'][i]);cut=m.Manifold()
  for key,c in g['RODS'].items():
   centre=[c['pivot_x'],0,c['pivot_z']]
   cut+=g['cyl'](2.65,-60,60,1,centre)
   if key=='write':
    bush=next(g['A'][j] for j,q in enumerate(g['P']) if q['id']=='bit write pivot bush 42')
    cut+=g['cyl'](3.95,-60,float(bush[:,1].max())+.25,1,centre)
  out=s-cut
  if s.volume()-out.volume()>.0001:
   g['A'][i]=g['triangles'](out);p['vertices']=len(g['A'][i])

def add_fixture_seating_lands(g):
 """Positive clamp stops keep the working fixtures at their designed depth."""
 schedule=json.loads((g['OUT']/'Frame fixture schedule.json').read_text())
 for record in schedule['fixtures']:
  i=next(i for i,p in enumerate(g['P']) if p['id']==record['part'])
  s=g['solid'](g['A'][i]);y=record['interface_y_mm']
  for fastener in record['fasteners']:
   c=fastener['centre_mm']
   s+=g['cyl'](3.8,y-.25,y+.2,1,c)-g['cyl'](3.3,y-.3,y+.3,1,c)
  g['A'][i]=g['triangles'](s);g['P'][i]['vertices']=len(g['A'][i])
  record['seating_plane_y_mm']=y+.2
  record['seating_land_radius_mm']=3.8
  record['seating_gap_mm']=0
 (g['OUT']/'Frame fixture schedule.json').write_text(json.dumps(schedule,indent=2))
