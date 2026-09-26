"""Verify actual paired pin socket material at detachable manufacturing joints."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
from wall_extra_connections import extra_connections
O=Path(__file__).resolve().parents[1]/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
ps={p['id']:p for p in P}
def mesh(n):
 p=ps[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def cyl(r,lo,hi,axis,c):
 s=m.Manifold.cylinder(hi-lo,r,circular_segments=32)
 if axis==0:s=s.rotate([0,90,0])
 elif axis==1:s=s.rotate([-90,0,0])
 pos=list(c);pos[axis]=lo;return s.translate(pos)
rows=[]
for e in extra_connections(O):
 solids=[mesh(e['part']),mesh(e['host'])];pins=[];centres=[]
 for pin in e['pins']:
  p=ps[pin['part']];axis=pin.get('axis',1);c=np.array(p['bounds']).mean(0);centres.append(c)
  fractions=[]
  intervals=pin.get('engagement_intervals_mm',[sorted([c[axis]+sign*1.5,c[axis]+sign*7.5]) for sign in [-1,1]])
  for lo,hi in intervals:
   ring=cyl(3.4,lo,hi,axis,c)-cyl(2.7,lo-.1,hi+.1,axis,c)
   fractions.append([(ring^s).volume()/ring.volume() for s in solids])
  # Either half may enter either piece, but they must engage opposite pieces.
  engaged=max(min(fractions[0][0],fractions[1][1]),min(fractions[0][1],fractions[1][0]))
  pins.append(dict(pin=pin['part'],socket_fractions=fractions,pass_check=p.get('lego_part') in ['2780','6558'] and engaged>.95))
 spacing=min((float(np.linalg.norm(a-b)) for i,a in enumerate(centres) for b in centres[i+1:]),default=0)
 rows.append(dict(**e,socket_checks=pins,min_pin_spacing_mm=spacing,pass_check=len(pins)>=2 and spacing>=7 and all(p['pass_check'] for p in pins)))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),connections=rows,connection_pass=bool(rows) and all(r['pass_check'] for r in rows),scope=__doc__,limitations=['Socket material and pin count are checked; insertion paths and print orientation are checked separately.'])
(O/'Revision connection checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
