"""Provide full axial clearance for the 1.6 mm central friction-pin collar."""
import json
from wall_extra_connections import extra_connections

def finish_pin_seats(g):
 P,A=g['P'],g['A'];lookup={p['id']:i for i,p in enumerate(P)};hosts={}
 for e in json.loads((g['OUT']/'Frame fixture schedule.json').read_text())['fixtures']:
  for pin in e['fasteners']:hosts[pin['part']]={e['part'],e['frame']}
 for key in ['clock','write']:
  for j in [1,2]:hosts[key+' rod coupler friction pin '+str(j)]={key+' pinned rod splice bridge',('Control '+key.upper()+' direct rod and pickup') if j==1 else ('bit '+key+' vertical control rod')}
 for e in extra_connections(g['OUT']):
  for pin in e['pins']:
   if P[lookup[pin['part']]].get('lego_part')=='2780':hosts[pin['part']]={e['part'],e['host']}
 cuts={};records=[]
 for pn,names in hosts.items():
  p=P[lookup[pn]];c=p['centre'];axis=p.get('axis',1)
  cut=g['cyl'](3.3,c[axis]-.9,c[axis]+.9,axis,c)
  for n in names:cuts[n]=cuts[n]+cut if n in cuts else cut
  records.append(dict(pin=pn,hosts=sorted(names),axis=axis,centre_mm=c,collar_clearance_depth_mm=1.8,collar_clearance_diameter_mm=6.6))
 for n,cut in cuts.items():
  i=lookup[n];A[i]=g['triangles'](g['solid'](A[i])-cut);P[i]['vertices']=len(A[i])
 (g['OUT']/'Pin collar seats.json').write_text(json.dumps(records,indent=2))
