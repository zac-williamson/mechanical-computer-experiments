"""Separate rear cheeks; reuse two cheek axes with 3L friction pins through all three pieces."""
import json
import numpy as np

def separate_fixture_bearing_faces(g):
 P,A=g['P'],g['A'];solid,triangles,box,cyl=[g[k] for k in ['solid','triangles','box','cyl']];records=[]
 for bank,offset,rot,module in [('master',[0,0,0],False,'bit'),('slave',[106,0,0],False,'bit'),('Control clock',[-116,-4,-104],True,'control'),('Control write',[-120,-12,-184],True,'control')]:
  def tr(s):return (s.rotate([0,-90,0]) if rot else s).translate(offset)
  def pt(c):return (np.array([-c[2],c[1],c[0]]) if rot else np.array(c))+offset
  region=tr(box([-28,-1,18.8],[35,18.60001,40.5]))
  probe=tr(cyl(3.4,14.8,18.4,1,[13.192323604,0,32.128448698])-cyl(2.9,14.7,18.5,1,[13.192323604,0,32.128448698]))
  ids=[i for i,p in enumerate(P) if p['kind']=='printed' and p.get('motion','fixed')=='fixed' and p['id'].startswith(module+' frame')]
  i=max(ids,key=lambda j:(solid(A[j])^probe).volume());host=solid(A[i]);cheek=host^region
  host-=tr(box([-28,-1,18.8],[35,19,40.5]));pins=[]
  for z in [28.4,35.6]:
   # Existing cheek column already clears the mechanism. Continue its bore
   # into the carrier; the longer pin retains front cheek, rear cheek, carrier.
   host+=tr(cyl(4.2,19,26,1,[-24,0,z]))
   host-=tr(cyl(2.5,18.9,26.1,1,[-24,0,z]))
   pn=bank+' cheek joining pin '+str(z);j=next(j for j,p in enumerate(P) if p['id']==pn)
   P.pop(j);A.pop(j)
   if j<i:i-=1
   c=pt([-24,13.8,z]);g['native'](pn,'6558',c.tolist(),axis=1,module=module)
   pins.append(dict(part=pn,centre_mm=c.tolist(),axis=1,engagement_intervals_mm=[[12.0+offset[1],18.1+offset[1]],[19.3+offset[1],25.3+offset[1]]]))
  assert len(cheek.decompose())==1 and len(host.decompose())==1,(bank,'disconnected cheek/carrier')
  A[i]=triangles(host);P[i]['vertices']=len(A[i]);name=bank+' removable rear bearing cheek';g['emit'](name,cheek,module=module,motion='fixed',color=P[i]['color'])
  records.append(dict(part=name,host=P[i]['id'],additional_hosts=[bank+' Front bearing cheek'],pins=pins,print_axis=1,print_up_sign=-1,bed_plane_mm=18.6+offset[1],assembly='Seat rear cheek on carrier. Insert long ends of two 3L pins into rear cheek and carrier; then press front cheek onto exposed short ends. Central collars remain between front and rear cheeks.'))
 (g['OUT']/'Print face additions schedule.json').write_text(json.dumps(dict(connections=records),indent=2))
