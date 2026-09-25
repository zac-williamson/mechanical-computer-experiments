"""Bring staggered transmission bearing rings to the wall's common print face.

Only rings are extended, rather than projecting the entire wall into neighbouring
mechanisms. Extensions have relieved bores and 45-degree lead-ins; original
bearing lands, shaft positions and two-pin mounting interfaces are retained.
"""
import json
import numpy as np
import manifold3d as m

def extend_bearing_rings(g):
 P,A=g['P'],g['A'];cyl,solid,triangles=g['cyl'],g['solid'],g['triangles']
 lookup={p['id']:i for i,p in enumerate(P)}
 schedule=json.loads((g['OUT']/'Bearing schedule.json').read_text());records=[]
 for wall in schedule['removable_walls']:
  i=lookup[wall['part']];axis='XYZ'.index(wall['bearing_axis']);s=solid(A[i]);bed=float(A[i][:,axis].min());entries=[]
  for b in schedule['bearings']:
   if b['module']!=wall['module']:continue
   arrays=np.concatenate([A[lookup[n]] for n in b['axle_parts']]);c=(arrays.min(0)+arrays.max(0))/2
   t=b['station_mm'];half=b['land_mm']/2;start=t-half
   ring=cyl(5,start,t+half,axis,c)-cyl(2.65,start-.1,t+half+.1,axis,c)
   if (ring^s).volume()<ring.volume()*.99:continue
   length=start-bed
   if length>.01:
    # Extra sleeve is clearance, not extra close-fit bearing length. A 45-degree
    # transition keeps the smaller original bore printable without a ledge.
    relief=min(.5,length);end=start-relief
    outer=cyl(4.95,bed,start+.02,axis,c)
    bore=cyl(3.2,bed-.01,end,axis,c) if end>bed-.01 else m.Manifold()
    taper=m.Manifold.cylinder(relief,2.7+relief,2.7,circular_segments=32)
    if axis==0:taper=taper.rotate([0,90,0])
    elif axis==1:taper=taper.rotate([-90,0,0])
    pos=c.copy();pos[axis]=end;taper=taper.translate(pos)
    bore+=taper+cyl(2.7,start-.00001,start+.03,axis,c)
    addition=(outer-bore)-s
    assert addition.volume()>0
    before=s;s+=outer-bore
    assert (before-s).volume()<1e-6
   entries.append(dict(shaft=b['shaft'],axis=b['axis'],centre_mm=c.tolist(),original_land_start_mm=start,original_land_mm=b['land_mm'],extension_mm=max(0,length),bed_plane_mm=bed,relieved_radius_mm=2.7+min(.5,max(0,length)) if length>.01 else 2.65))
  if any(e['extension_mm']>.01 for e in entries):A[i]=triangles(s);P[i]['vertices']=len(A[i])
  records.append(dict(part=wall['part'],axis=wall['bearing_axis'],print_up_sign=1,bed_plane_mm=bed,bearings=entries))
 # The CLOCK pivot fixture also has a 0.4 mm step at its seating lands.
 # Add a short bed foot and move its outer bush by the same amount, preserving
 # 0.2 mm clearance. The 4 mm bush retains 3.6 mm axle engagement.
 c=np.array([-96.,0.,40.])
 probe=cyl(3.5,36.5,37.7,1,c)-cyl(2.85,36.4,37.8,1,c)
 ids=[i for i,p in enumerate(P) if p['id'].startswith('bit frame 0 removable fixture')]
 i=max(ids,key=lambda j:(solid(A[j])^probe).volume())
 foot=cyl(4.5,37.7,38.2,1,c)-cyl(2.85,37.6,38.3,1,c)
 A[i]=triangles(solid(A[i])+foot);P[i]['vertices']=len(A[i])
 j=lookup['bit clock pivot bush 40'];A[j]=A[j]+np.array([0,.4,0]);P[j]['centre'][1]+=.4
 P[j]['id']='bit clock pivot bush 40.4'
 # The WRITE pivot now shares the lower-guide carrier. Its rear bearing
 # ring must meet that carrier's seating-plane bed face as well.
 c=np.array([-104.,0.,8.]);probe=cyl(3.5,36.5,37.7,1,c)-cyl(2.85,36.4,37.8,1,c)
 i=max(ids,key=lambda j:(solid(A[j])^probe).volume())
 foot=cyl(4.5,37.7,38.2,1,c)-cyl(2.85,37.6,38.3,1,c)
 A[i]=triangles(solid(A[i])+foot);P[i]['vertices']=len(A[i])
 # Move the three upstream collars to the new outside face. Preserve the
 # original 0.2 mm axial endplay; collars must not rotate inside a print foot.
 moved=[]
 for name,shift,shaft,face in [('M output axial stop 30.2',-3.4,'M output',29.0),('Feedback axial stop 30.2',-3.4,'Feedback',29.0),('Slave worm axial stop 128.6',-.4,'Slave worm',130.4)]:
  i=lookup[name];A[i]=A[i]+np.array([shift,0,0]);P[i]['centre'][0]+=shift
  newname=shaft+' axial stop '+str(round(P[i]['centre'][0],3));P[i]['id']=newname
  for row in schedule['retention']:
   if row['shaft']==shaft:
    stop=next(x for x in row['stops'] if x['side']==-1)
    stop['centre']+=shift;stop['contact_face_mm']=face;stop['part']=newname
    stop['print_foot_contact']=True
  moved.append(dict(part=newname,old_part=name,shift_mm=shift,contact_face_mm=face,endplay_mm=.2))
 (g['OUT']/'Bearing schedule.json').write_text(json.dumps(schedule,indent=2))
 (g['OUT']/'Bearing bed-face schedule.json').write_text(json.dumps(dict(walls=records,moved_collars=moved,scope=__doc__),indent=2))
