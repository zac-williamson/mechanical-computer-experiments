from common import *
from shapely.ops import unary_union
parts={n:load(n) for n in ['Left carriage half','Right carriage half','Direct lever and band cleat','Front actuator bridge','Rear bridge and band anchor']}
from shapely.geometry import Point
pad=unary_union([Point(*(pv+[-8,-4])).buffer(2.6,quad_segs=64),Point(*(pv+[8,-4])).buffer(2.6,quad_segs=64)])
pads=extr(pad,35.79,7.41);body=parts['Direct lever and band cleat']-pads
worm=m.Manifold.cylinder(26.,5.5,circular_segments=128,center=True).rotate([0,90,0]).translate([0,10.2,32]);hits=[]
# Full angular envelope, not only the animated states. Frame and worm envelope checks.
for beta in np.arange(-28,28.01,.25):
 a=parts['Direct lever and band cleat'].translate([-pv[0],-pv[1],0]).rotate([0,0,float(beta)]).translate([*pv,0])
 for name,b in [('worm full rotation/travel envelope',worm),('Front actuator bridge',parts['Front actuator bridge']),('Rear bridge and band anchor',parts['Rear bridge and band anchor']),('full bushes',cyl(3.95,32.2,48.2))]:
  vol=(a^b).volume()
  if vol>.001:hits.append(dict(beta=float(beta),component=name,volume=vol))
print('full angle envelope hits',len(hits),hits[:5],flush=True)
assert not hits, hits[:10]
rows=[];tested=0;blocked=0
for beta in np.arange(-28,28.01,.5):
 a=body.translate([-pv[0],-pv[1],0]).rotate([0,0,float(beta)]).translate([*pv,0]);pp=pads.translate([-pv[0],-pv[1],0]).rotate([0,0,float(beta)]).translate([*pv,0])
 for q in np.linspace(-4.6,4.575,93):
  tested+=1
  for name in ['Left carriage half','Right carriage half']:
   car=parts[name].translate([float(q),0,0]);padvol=(pp^car).volume();vol=(a^car).volume()
   if padvol>.001:blocked+=1
   if vol>.001:rows.append(dict(beta=float(beta),q=float(q),part=name,body_volume=vol,pad_volume=padvol,already_blocked_by_pad=padvol>.001))
print('cross product',tested,'body contacts',len(rows),'before pad contact',sum(not r['already_blocked_by_pad'] for r in rows),flush=True)
assert not any(not r['already_blocked_by_pad'] for r in rows), rows[:10]
