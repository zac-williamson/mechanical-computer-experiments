"""Front-open rod guides with removable, two-pin keeper caps.

Rods enter sideways before caps are fitted; no enlarged rod end passes through
an enclosed ring. Cap pin axes are normal to their broad print-bed face.
"""
import json
import numpy as np
import manifold3d as m
from wall_pose import example_frames, vertices, joint, transform
from wall_flat_frame import native_envelope

GUIDES=[('bit','clock',-116,10,[-22,21.5]),('bit','write',-128,26,[-12,32]),('control','clock',-116,10,[-132,-72]),('control','write',-128,26,[-204,-140,-72])]

def open_rod_guides(g):
 P,A=g['P'],g['A'];box,cyl,solid,triangles,emit,native=[g[k] for k in ['box','cyl','solid','triangles','emit','native']]
 cases=json.loads((g['BASE']/'Compact contact-resolved operation.json').read_text())['cases']
 frames=[c['frames'][i] for c in cases for i in [0,len(c['frames'])//2,-1]]
 obstacle=[]
 def resolve(s):return s[0].transform(s[1]) if isinstance(s,tuple) else s
 for p,a in zip(P,A):
  if p['kind']=='elastic' or a[:,0].min()>-80 or a[:,0].max()<-175:continue
  if p['kind']=='native':
   base=native_envelope(p,a,joint,frames[0]);bb=np.array(base.bounding_box()).reshape(2,3);centre=bb.mean(0);samples=[];bounds=[];seen=set()
   for f in frames:
    tf=transform(p,f);d=tf[:3,:3]@centre+tf[:3,3]-centre;k=tuple(d)
    if k in seen:continue
    seen.add(k);tf=np.eye(4);tf[:3,3]=d;samples.append((base,tf[:3]));bounds.append(bb+d)
  elif p.get('motion','fixed')=='fixed':
   samples=[solid(a)];bounds=[np.array([a.min(0),a.max(0)])]
  else:
   base=solid(a);samples=[];bounds=[];seen=set()
   for f in frames:
    tf=transform(p,f);k=tuple(tf.ravel())
    if k in seen:continue
    seen.add(k);v=a@tf[:3,:3].T+tf[:3,3];samples.append((base,tf[:3]));bounds.append(np.array([v.min(0),v.max(0)]))
  obstacle.append((p,samples,bounds))
  if p['module']=='coupler':
   delta=np.array([0,0,112.]);ss=[]
   for entry in samples:
    if isinstance(entry,tuple):
     mat=entry[1].copy();mat[:,3]+=delta;ss.append((entry[0],mat))
    else:ss.append(entry.translate(delta))
   obstacle.append((dict(p,module='bit',id=p['id']+' next row'),ss,[b+delta for b in bounds]))
 def conflicts(s,module,ignore):
  b=np.array(s.bounding_box()).reshape(2,3);hits=[]
  for p,ss,bs in obstacle:
   if p['module'] not in [module,'coupler'] or p['id'] in ignore:continue
   for q,qb in zip(ss,bs):
    if np.all(np.minimum(b[1],qb[1])-np.maximum(b[0],qb[0])>1e-5):
     v=(s^resolve(q)).volume()
     if v>.005:hits.append((p['id'],round(v,4)));break
  return hits
 limits={mod:np.array([np.concatenate([a for p,a in zip(P,A) if p['module']==mod]).min(0),np.concatenate([a for p,a in zip(P,A) if p['module']==mod]).max(0)]) for mod in ['bit','control']}
 limits['bit'][0,0]-=6.8  # Small keeper overhang; frame and vertical pitch stay unchanged.
 records=list(g.get('guide_seed_records',[]))
 for module,key,x,y,zs in GUIDES:
  for z in zs:
   probe=box([x-3,y-2,z-1],[x+3,y-.4,z+1])
   ids=[i for i,p in enumerate(P) if p['module']==module and p['kind']=='printed' and p.get('motion','fixed')=='fixed']
   i=max(ids,key=lambda j:(solid(A[j])^probe).volume());host=P[i]['id'];original=solid(A[i]);half=(3 if z==-22 else 2) if module=='bit' and key=='clock' else (5 if module=='bit' else 2)
   # Entire front wall is removed, including any support-free projection behind it.
   opening=box([x-3.4,y-2.01,z-half-.01],[x+3.4,y-.399,z+half+.01]);opened=original-opening
   success=None;fail=[]
   layouts=[(90,side,dz) for side in [-13.5,15.5,13.5,11,-11] for dz in [9,-11,10,-10,-9,11,-7,0,7,-14,14]]+[(angle,shift,0) for angle,shift in [(90,-11),(90,11),(-45,0),(-35,0),(-55,0),(0,0),(90,0),(45,0)]]
   preferred={('bit','clock',-22):(90,-13.5,9),('bit','clock',21.5):(90,-13.5,7),('bit','write',-12):(90,-8.5,-6.5),('bit','write',32):(90,-8.5,3),('control','clock',-132):(90,15.5,9),('control','clock',-72):(90,15.5,-11),('control','write',-204):(0,0,-9),('control','write',-140):(90,15.5,-10),('control','write',-72):(90,-13.5,9)}
   if (module,key,z) in preferred:layouts=[preferred[module,key,z]]+layouts
   for angle,shift,zshift in layouts:
    for radius in [7,9,11,13]:
     dx,dz=radius*np.cos(np.deg2rad(angle)),radius*np.sin(np.deg2rad(angle));centres=[(x+shift-dx,z+zshift-dz),(x+shift+dx,z+zshift+dz)]
     # Rounded socket ears keep >=2mm wall outside the 3.3mm collar recess.
     ears=None;cap=None
     for xx,zz in centres:
      ear=cyl(5.3,y-.2,y+7.6,1,[xx,0,zz]);lip=cyl(5.3,y-8.2,y-.6,1,[xx,0,zz])
      ear+=box([xx-2.6,y-.2,min(zz,z)-2.6],[xx+2.6,y+7.6,max(zz,z)+2.6])
      ear+=box([min(xx,x)-2.6,y-.2,z-min(half,2.6)],[max(xx,x)+2.6,y+7.6,z+min(half,2.6)])
      bridge=(lip+box([x-3.4,y-8.2,z-half],[x+3.4,y-.6,z+half])).hull()
      ears=ear if ears is None else ears+ear;cap=bridge if cap is None else cap+bridge
     # Retain nominal 0.4mm rear/lateral rod clearance, 0.6mm front clearance.
     ears-=box([x-3.4,y-20,z-30],[x+3.4,y+6.4,z+30])
     # Actual moving rod and splice envelopes below govern clearance; do not
     # carve a wide shoe-sized channel through every mounting ear.
     for xx,zz in centres:
      bore=cyl(2.5,y-8.3,y+7.7,1,[xx,0,zz])+cyl(3.3,y-.8,y,1,[xx,0,zz]);ears-=bore;cap-=bore
     bb=np.array((ears+cap).bounding_box()).reshape(2,3)
     if np.any(bb[0]<limits[module][0]-.001) or np.any(bb[1]>limits[module][1]+.001):continue
     # Trim only added material around nearby fixed structure, with clearance.
     for op,oss,obs in obstacle:
      if op['module']!=module or op['id']==host or op.get('motion','fixed')!='fixed' or op['kind']!='printed':continue
      eb=np.array((ears+cap).bounding_box()).reshape(2,3)
      if not np.all(np.minimum(eb[1],obs[0][1])-np.maximum(eb[0],obs[0][0])>0):continue
      keep=oss[0]
      for ax in range(3):
       d=np.zeros(3);d[ax]=.35;keep+=oss[0].translate(d)+oss[0].translate(-d)
      ears-=keep;cap-=keep
     if module=='control' and key=='write' and z==-204:
      # Recess the front of the bridge above its lower pin row, leaving
      # the rear keeper face clear of the strengthened carriage's full sweep.
      for op,oss,obs in obstacle:
       if op['id']=='Control write Carriage fork and roof':
        bb=np.array(obs);clearance=box(bb[:,0,:].min(0)-.3,bb[:,1,:].max(0)+.3)
        cap-=clearance;ears-=clearance
     if module=='control' and key=='write' and z==-72:
      # Relief on a stationary keeper wing for the complete splice travel.
      for op,oss,obs in obstacle:
       if op['id']=='write pinned rod splice bridge':
        bb=np.array(obs);clearance=box(bb[:,0,:].min(0)-.3,bb[:,1,:].max(0)+.3)
        cap-=clearance;ears-=clearance
     body=(opened-cap-cap.translate([0,.2,0]))+ears
     for xx,zz in centres:body-=cyl(2.5,y-8.3,y+7.7,1,[xx,0,zz])+cyl(3.3,y-.8,y,1,[xx,0,zz])
     # Discard sub-0.1 mm³ chips left by clearance trimming; retain every substantive solid.
     body=sum((s for s in body.decompose() if s.volume()>.1),m.Manifold())
     cap=sum((s for s in cap.decompose() if s.volume()>.1),m.Manifold())
     if len(body.decompose())!=1 or len(cap.decompose())!=1:
      continue
     hits=conflicts((ears-original)+cap,module,{host})
     if hits:fail.append((angle,shift,zshift,radius,hits));continue
     if (cap^body).volume()>.005:continue
     socket_ok=True
     for xx,zz in centres:
      for ss,lo,hi in [(cap,y-7.9,y-1.9),(body,y+1.1,y+7.1)]:
       ring=cyl(3.4,lo,hi,1,[xx,0,zz])-cyl(2.7,lo-.1,hi+.1,1,[xx,0,zz])
       if (ring^ss).volume()/ring.volume()<.97:socket_ok=False
     for xx,zz in centres:
      for ss,lo,hi in [(cap,y-.75,y-.65),(body,y+.05,y+.15)]:
       ring=cyl(4.7,lo,hi,1,[xx,0,zz])-cyl(3.35,lo-.01,hi+.01,1,[xx,0,zz])
       if (ring^ss).volume()/ring.volume()<.99:socket_ok=False
     if not socket_ok:continue
     success=body,cap,centres;break
    if success:break
   if not success:
    if g.get('debug_guide_checkpoint'):
     import pickle
     pickle.dump((P,A,records,fail),open(g['debug_guide_checkpoint'],'wb'))
    raise ValueError(('No rod cap layout',module,key,z,fail))
   print('GUIDE',module,key,z,'layout',angle,shift,zshift,radius,flush=True)
   body,cap,centres=success;A[i]=triangles(body);P[i]['vertices']=len(A[i]);name=f'{module} {key} rod guide cap {z:g}'
   emit(name,cap,module=module,color=(.38,.57,.53),print_axis=1,print_up_sign=1)
   pins=[]
   for j,(xx,zz) in enumerate(centres):
    pin=f'{name} friction pin {j+1}';native(pin,'2780',[xx,y-.4,zz],axis=1,module=module);pins.append(dict(part=pin,centre_mm=[xx,y-.4,zz]))
   records.append(dict(part=name,host=host,module=module,control=key,guide_centre_mm=[x,y,z],guide_half_length_mm=half,pins=pins,axis='Y',print_up_sign=1,bed_plane_mm=y-8.2,seating_face_mm=y-.6,pin_interface_mm=y-.4,assembly='On the detached carrier, remove cap. CLOCK enters from front (-Y). WRITE enters sideways at Y=17.8, then seats toward +Y: from right for controller guide -204, from left for other WRITE guides. Fit carriers to rods before attaching to frame and fitting followers/actuator hardware. Seat pins from mating faces, then press cap onto exposed halves.'))
   # Later caps must also clear earlier caps and their new support ears.
   obstacle=[row for row in obstacle if row[0]['id']!=host]
   obstacle.append((P[i],[body],[np.array(body.bounding_box()).reshape(2,3)]))
   obstacle.append((P[-3],[cap],[np.array(cap.bounding_box()).reshape(2,3)]))
   for pp,aa in zip(P[-2:],A[-2:]):
    ss=native_envelope(pp,aa,joint,frames[0]);obstacle.append((pp,[ss],[np.array(ss.bounding_box()).reshape(2,3)]))
 (g['OUT']/'Rod guide assembly schedule.json').write_text(json.dumps(dict(connections=records,scope=__doc__),indent=2))
