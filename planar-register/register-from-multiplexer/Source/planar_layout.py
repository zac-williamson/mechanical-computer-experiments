"""Shallow XZ register development: direct slotted bell crank, no rear cam layer."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];C=R/'Core layout';L=R/'Latch coupon';O=R/'Planar register';O.mkdir(exist_ok=True)
def so(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 z=s.to_mesh64();return trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=64)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 t=list(c);t[axis]=a;return s.translate(t)
def load(p):return so(trimesh.load(p))
def place(s):return s.rotate([90,0,0]).translate([0,30,-34])
meta=json.loads((C/'printed-parts.json').read_text());P={p['id']:load(C/(p['id']+'.stl')) for p in meta}
# Keeper is integral with the existing moving bearing support, below the gears.
n='Memory — Right carriage bearing support';old=P[n]
P[n]+=place(load(L/'Memory keeper.stl'))+box([11.4,32.8,-29.3],[19.2,38,8.7])+box([11.4,24.4,-.3],[19.2,38,8.7])+box([11.4,26,-34],[19.2,38,-29.2])
P[n]-=cy(3.3,8,15.6,0,[0,24,3.5])-old
P[n]-=cy(2.45,15.59,22,0,[0,24,3.5])
front=place(load(L/'Front guide.stl'));rear=place(load(L/'Rear guide.stl'))
front+=box([-6,18,-46.8],[6,30,-34.4])-box([-2.35,21.2,-46.9],[2.35,26.8,-34.3])
# Arm passage remains open across the full memory stroke.
window=box([6.4,24,-29.6],[25.4,30.2,9.1]);front-=window;rear-=window
# Integral lower guide and shallow rear posts; upper guide retains two LEGO pins.
base=P['Memory — Common baseboard']+P['Write — Common baseboard']+box([33.8,30.4,-4],[66.8,38.4,8])
base-=box([6.4,32.4,-29.6],[24.2,38.5,8.8])
base-=box([-34.4,30,-31.6],[34.4,34.4,-24.4])
for a,b in [(-34,-26),(26,34)]:
 base+=box([a,34.4,-38.4],[b,38.4,-24])+box([a,33.6,-38.4],[b,38.4,-32])
bolt=place(load(L/'Sliding bolt.stl')).translate([0,0,-1.8])
bolt-=box([-12,13,-56],[-7.2,31,-47.7])+box([7.2,13,-56],[12,31,-47.7])
bolt+=box([-7.2,13.2,-55.8],[7.2,26.4,-47.8])
# Bell crank lies in front of the latch, all within the original Y envelope.
# Pivot (20,*, -52.8), output radius20 left, input radius10 down.
rock=box([0,5.6,-56.8],[20,13.2,-48.8])+box([16,5.6,-62.8],[24,13.2,-52.8])
for x,z,r in [(0,-52.8,4),(20,-52.8,5.6),(20,-62.8,4)]:rock+=cy(r,5.6,13.2,1,[x,0,z]);rock-=cy(2.6,5.5,13.3,1,[x,0,z])
# Native half-bush roller runs in a vertical slot: horizontal write travel directly turns the crank.
w='Write — Right carriage bearing support'
P[w]+=box([81.2,24.6,-67],[88.8,29.8,0])+box([16,23.6,-67],[88.8,29.8,-59])
P[w]+=box([12.4,13.2,-69],[27.6,29.8,-55.6])
P[w]-=box([16.2,13.1,-66.6],[23.8,23.2,-56.6])
# Open pedestal, integral with common baseboard. It does not create another depth layer.
base+=box([40,34.4,-74],[48,38.4,-20])+box([33.6,34.4,-26.7],[48,38.4,-20])+box([14.4,.8,-74],[48,38.4,-70])
base+=box([31.2,.8,-74],[38.8,5.2,-44.4])+box([14.4,.8,-48.4],[38.8,5.2,-44.4])+box([14.4,.8,-56.4],[25.8,5.2,-46.8])
base+=box([14.4,30.2,-74],[25.8,34.6,-46.8])
for ya,yb in [(.8,5.2),(30.2,34.6)]:base-=cy(2.6,ya-.1,yb+.1,1,[20,0,-52.8])
base+=front
base-=box([6.4,30,-34.4],[24.2,38.5,8.8])
rear-=box([6.4,30,-34.4],[24.2,38.5,8.8])
for n in ['Memory — Common baseboard','Write — Common baseboard']:del P[n]
meta=[p for p in meta if p['id'] not in ['Memory — Common baseboard','Write — Common baseboard']]
for n,s,mo in [('Common planar base',base,'fixed'),('Upper lock guide',rear,'fixed'),('Lock bolt',bolt,'bolt'),('Direct bell crank',rock,'bell-crank')]:P[n]=s;meta.append(dict(id=n,bank='Planar',motion=mo,source='new'))
for p in meta:
 t=mesh(P[p['id']]);t.export(O/(p['id']+'.stl'));p.update(bounds=t.bounds.tolist(),solids=len(t.split()),watertight=bool(t.is_watertight))
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
# Hardware retained from the two core layout. New rollers/axles below are true source meshes.
hm=json.loads((C/'hardware.json').read_text());v=np.load(C/'hardware.npz')['vertices'];arr=[v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hm]
def template(n):
 h=next(h for h in hm if h['id']==n);a=v[h['offset']//3:h['offset']//3+h['vertices']].copy();return a-(a.min(0)+a.max(0))/2
bush=template('Memory — A-input-bush-10')[:,[1,0,2]];ax=template('Memory — pivot-stop-axle');pin=template(next(h['id'] for h in hm if h['id'].startswith('Memory — Baseboard pin')))
def add(n,a,c,mo,part):
 a=a+c;hm.append(dict(id=n,bank='Planar',source='new',motion=mo,lego_part=part,offset=sum(x.size for x in arr),vertices=len(a)));arr.append(a)
for x,z,mo in [(0,-52.8,'bell-crank'),(20,-62.8,'bell-crank'),(20,-52.8,'fixed')]:
 
 if mo=='fixed':
  a=template('Memory — A-idler-shaft')[:,[1,0,2]];a[:,1]*=40/np.ptp(a[:,1]);add('Planar pivot 5L axle — approximate ends',a,[x,16.8,z],mo,'32073');add('Planar rear pivot bush',bush,[x,37,z],mo,'32123a')
 else:
  a=template('Memory — A-idler-shaft')[:,[1,0,2]];a[:,1]*=16/np.ptp(a[:,1]);add('Planar 2L axle — approximate ends '+str(z)+str(x),a,[x,9.6,z],mo,'32062')
 add('Planar front bush '+str(z)+str(x),bush,[x,2.8 if mo=='bell-crank' else -1.2,z],mo,'32123a')
 if mo=='bell-crank':add('Planar roller '+str(x),bush,[x,15.6,z],mo,'32123a')
for x in [-30,30]:
 # Lower guide integral with base; no extra mount pins.
 add('Planar guide joint '+str(x),pin[:,[0,2,1]],[x,24,-31.6],'fixed','2780')
(O/'hardware.json').write_text(json.dumps(hm,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr))
print('Solids:',[(p['id'],p['solids']) for p in meta if p['solids']!=1]);bb=np.array([p['bounds'] for p in meta]);print('Printed Y range',bb[:,0,1].min(),bb[:,1,1].max())
