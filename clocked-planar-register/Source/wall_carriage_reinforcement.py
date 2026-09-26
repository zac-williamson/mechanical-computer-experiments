"""Local reduced-rail carriage reinforcement, adapted from the accepted planar fix.

Work in each core's canonical coordinates before controller rotation/chassis
consolidation. Gear, axle, pin and actuator datums stay fixed.
"""
import numpy as np

def revise(name,a,g):
 box,cyl,solid,triangles=[g[k] for k in ['box','cyl','solid','triangles']]
 bank=name.split()[0]
 if bank not in ['master','slave','clock','write']:return a
 x,z,flip={'master':(0,0,False),'slave':(106,0,False),'clock':(40,-16,True),'write':(-76,-16,True)}[bank]
 sign=np.array([-1,1,-1]) if flip else np.ones(3)
 local=(a-np.array([x,0,z]))*sign
 s=solid(local)
 if name.endswith(('Carriage fork and roof','Right carriage bearing support')):
  b=np.array(s.bounding_box()).reshape(2,3)
  xa,xb=(b[0,0],b[1,0]) if name.endswith('Right carriage bearing support') else (b[0,0],-8)
  rear=29.7 if bank=="clock" else 32.5
  add=box([xa,11.15,7.35],[xb,16.65,13.85])
  add+=box([xa,16.6,4.7],[xb,24.4,9.35])
  add+=box([xa,16.6,11.85],[xb,rear,15.9])+box([xa,20,5.3],[xb,rear,9.35])
  add-=cyl(9.15,xa-.1,xb+.1,0,[0,10.2,0])
  add-=cyl(2.85,xa-.1,xb+.1,0,[0,10.2,16])
  add-=cyl(4.05,xa-.1,xb+.1,0,[0,10.2,16])-s
  for yy,zz in [(24,3.5),(23.95,19.5)]:add-=cyl(3.65,xa-.1,xb+.1,0,[0,yy,zz])-s
  if bank=="slave" and name.endswith("Carriage fork and roof"):
   add-=box([xa-.01,27.9,11.8],[-12,32.6,16.0])
  s+=add
 elif name.endswith('carriage track'):
  # The wall variant's backing blocks started at Y30.2, unlike the planar
  # frame. Set their front face behind the new pads at Y32.8.
  s-=box([-40,15.5,6.7],[40,32.8,9.6])+box([-40,15.5,11.6],[40,32.8,14.5])
  s-=box([-40,15.5,9.59],[40,17.05,11.61])
  # Continuous backing for both carriage halves throughout their stroke.
  s+=box([-16.4 if bank=="slave" else -20,32.8,11.85],[20,37.8,15.9])
 elif name.endswith('compact bolt guide'):
  # Original bridge root blocks slipping a closed band over the front lip.
  # Keep the existing seat and band route, move its root behind the groove.
  bx=-5.05
  s-=box([bx-3.01,35.69,46.21],[bx+3.01,38.91,52.41])
  s+=box([bx-3,36,45.5],[bx+3,40.4,46.2])
  s+=box([bx-3,37.9,46.19],[bx+3,40.4,50])
  for lo,hi,r in [(35.7,36.7,2.4),(36.7,37.9,1.8),(37.9,40.4,2.4)]:s+=cyl(r,lo,hi,1,[bx,0,50])
 else:return a
 return triangles(s)*sign+np.array([x,0,z])


def reopen_band_access(g):
 # Apply after frame projections: no subsequent frame union may close access.
 for bx in [-5.05,100.95]:
  # The chassis splitter otherwise classifies the shallow peg as frame material.
  # Attach the complete rear-rooted hook to the removable bolt-guide fixture.
  foot=g['box']([bx-3,36,45.5],[bx+3,37.79,46.2])
  candidates=[i for i,p in enumerate(g['P']) if p['kind']=='printed' and p['module']=='bit' and 'removable fixture' in p['id']]
  host=max(candidates,key=lambda i:(g['solid'](g['A'][i])^foot).volume())
  hook=g['box']([bx-3,36,45.5],[bx+3,40.4,46.2])+g['box']([bx-3,37.9,46.19],[bx+3,40.4,50])
  for lo,hi,r in [(35.7,36.7,2.4),(36.7,37.9,1.8),(37.9,40.4,2.4)]:hook+=g['cyl'](r,lo,hi,1,[bx,0,50])
  joined=g['solid'](g['A'][host])+hook
  assert len(joined.decompose())==1,'Band hook must join removable guide'
  g['A'][host]=g['triangles'](joined);g['P'][host]['vertices']=len(g['A'][host])
  pocket=g['cyl'](3.5,33.0,37.9,1,[bx,0,50])
  pocket-=g['cyl'](2.4,35.7,36.7,1,[bx,0,50])+g['cyl'](1.8,36.7,38,1,[bx,0,50])
  for i,p in enumerate(g['P']):
   if p['kind']=='printed' and p['module']=='bit' and p.get('motion','fixed')=='fixed':
    a=g['A'][i]
    if a[:,0].max()<bx-3.5 or a[:,0].min()>bx+3.5:continue
    cut=g['box']([bx-3.5,33,45.3],[bx+3.5,40.6,53.5]) if 'coordinated chassis' in p['id'] else pocket
    g['A'][i]=g['triangles'](g['solid'](a)-cut);p['vertices']=len(g['A'][i])


def seat_revised_cap(g):
 # Retain a small assembly gap after successive host triangulations.
 name='control write rod guide cap -204'
 import json
 rows=json.loads((g['OUT']/'Rod guide assembly schedule.json').read_text())['connections']
 row=next(x for x in rows if x['part']==name)
 ids={p['id']:i for i,p in enumerate(g['P'])};i=ids[name];j=ids[row['host']]
 cap=g['solid'](g['A'][i]);clear=cap
 for axis in range(3):
  d=np.zeros(3);d[axis]=.06;clear+=cap.translate(d)+cap.translate(-d)
 body=g['solid'](g['A'][j])-clear
 # Open the front of the shank insertion corridor through the residual lip.
 body-=g['box']([-131.4,14,-206.01],[-124.6,25.7,-201.99])
 pieces=[part for part in body.decompose() if part.volume()>.1]
 assert len(pieces)==1,[(part.volume(),part.bounding_box()) for part in pieces]
 body=pieces[0]
 g['A'][j]=g['triangles'](body);g['P'][j]['vertices']=len(g['A'][j])
