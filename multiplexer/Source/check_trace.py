from common import *
O=ROOT
parts={n:load(n) for n in ['Left carriage half','Right carriage half','Direct lever and band cleat']}
frame={n:solid(trimesh.load(O/(n+'.stl'))) for n in ['Front actuator bridge','Rear bridge and band anchor','Front bottom guide','Base','Bearing wall X-4','Bearing wall X28','Bearing wall X26','Bearing wall X-28 four holes','Bearing wall X-28 three holes','A idler bearing']}
records=[]
for q in np.linspace(-4.6,4.575,93):
 for n in ['Left carriage half','Right carriage half']:
  a=parts[n].translate([float(q),0,0])
  for name,b in {**frame,'full pivot bushes':cyl(5.35,32.2,48.2),'pivot axle envelope':cyl(3.,10.4,58.4)}.items():
   vol=(a^b).volume()
   if vol>.001:records.append(dict(a=n,b=name,q=float(q),v=vol))
print('carriage-frame/pivot hits',len(records),records[:5],flush=True)
rows=json.loads((O/'Switching trace.json').read_text())['frames'];seen=set();hits=[]
for r in rows:
 q,beta=r['q'],r['b']
 if (q,beta) in seen:continue
 seen.add((q,beta));a=parts['Direct lever and band cleat'].translate([-pv[0],-pv[1],0]).rotate([0,0,beta]).translate([*pv,0])
 for n,b in {**frame,**{n:parts[n].translate([q,0,0]) for n in ['Left carriage half','Right carriage half']},'pivot bushes':cyl(3.95,32.2,48.2)}.items():
  vol=(a^b).volume()
  if vol>.001:hits.append(dict(a='lever',b=n,q=q,beta=beta,v=vol))
print('lever full-body hits',len(hits),hits[:8],flush=True)
assert not records, records[:10]
assert all(r['b']=='Left carriage half' and r['v']<.005 for r in hits), hits[:10]
