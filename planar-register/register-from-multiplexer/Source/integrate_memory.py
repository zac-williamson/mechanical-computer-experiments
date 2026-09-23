"""Integrate the compact rear lock into the memory core without modifying its
worm, reaction gear, lever, clutch fork or original guide contact surfaces.
The write-to-lock cam is a separate, unfinished integration item.
"""
from pathlib import Path
import json,shutil,itertools,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];B=R.parent/'work/register-mux-reference/multiplexer';C=R/'Core layout';L=R/'Latch coupon';O=R/'Assembly';O.mkdir(exist_ok=True)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 z=s.to_mesh64();return trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=80)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 t=list(c);t[axis]=a;return s.translate(t)
def load(p):return solid(trimesh.load(p))
def place(s):return s.rotate([90,0,0]).translate([0,52,20.9])
meta=json.loads((C/'printed-parts.json').read_text());P={p['id']:load(C/(p['id']+'.stl')) for p in meta}
# Wide flat keeper with an upright stiffening rib; print flat on Z=20.9.
keeper=place(load(L/'Memory keeper.stl'))+box([11.4,26.8,20.9],[19.2,34.4,30.5])+box([11.4,34.4,20.9],[20.4,46.4,30.5])
# Preserve all bearing geometry forward of Y14.4. This is an attachment change only.
rn='Memory — Right carriage bearing support';old=P[rn]
foot=(old-box([7.9,26.8,20.5],[30,40,40]))+box([8,26.8,12.9],[19.2,34.4,20.5])+box([11.4,34.4,12.9],[20.4,46.4,20.5])
# Original joining pin bore and its actual native collar void; do not fill the joint.
void=cyl(3.3,8,15.6,0,[0,23.95,19.5])-old
for name in ['foot','keeper']:
 s=locals()[name]-void-cyl(2.45,15.59,17.3,0,[0,23.95,19.5])
 for y in [30.4,42.4]:
  s-=cyl(2.45,12.8,30.6,2,[15.,y,0])
  s-=cyl(3.2,20.2,21.2,2,[15.,y,0])
 locals()[name]=s
P[rn]=foot;P['Memory — Keeper and rear arm']=keeper
front=place(load(L/'Front guide.stl'));rear=place(load(L/'Rear guide.stl'));bolt=place(load(L/'Sliding bolt.stl'))
# Stem must pass through the near guide rail, independently of both carriage states.
window=box([6.4,28.0,12.5],[25.4,46.8,30.9]);front-=window;rear-=window
# Flat two-pin interface to the back of the main baseboard.
for xa,xb in [(-34,-26),(26,34)]:
 front-=box([xa-.01,30,16],[xb+.01,38.8,32]);rear-=box([xa-.01,30,16],[xb+.01,38.8,32])
 front+=box([xa,38.8,-1],[xb,46.8,22.9])
for x in [-30,30]:
 front-=cyl(2.45,38.7,47,1,[x,0,3.])+cyl(3.2,38.7,39.1,1,[x,0,3.])
 front-=cyl(2.45,14.9,23.1,2,[x,46,0])+cyl(3.2,22.5,23.1,2,[x,46,0])
bn='Memory — Common baseboard';base=P[bn]-window-box([3.,28,12.5],[25.4,34.8,30.9])
base-=box([-26.4,35.6,16.1],[26.4,38.5,31.3])
for x in [-30,30]:base-=cyl(2.45,30.3,38.5,1,[x,0,3.])+cyl(3.2,38.1,38.5,1,[x,0,3.])
P[bn]=base;P['Memory — Fixed lower latch guide']=front;P['Memory — Removable upper latch guide']=rear;P['Memory — Lock bolt']=bolt
added=[dict(id='Memory — Keeper and rear arm',bank='Memory',source='new',motion='carriage'),dict(id='Memory — Fixed lower latch guide',bank='Memory',source='new',motion='fixed'),dict(id='Memory — Removable upper latch guide',bank='Memory',source='new',motion='fixed'),dict(id='Memory — Lock bolt',bank='Memory',source='new',motion='bolt')];meta+=added
# A single printed joining base avoids screws and additional structural joints.
ew='Write — Common baseboard'
joined=P[bn]+P[ew]+box([33.8,30.4,-4],[66.8,38.4,8])
del P[bn];del P[ew];meta=[p for p in meta if p['id'] not in [bn,ew]]
P['Common two-core baseboard']=joined;meta.append(dict(id='Common two-core baseboard',bank='shared',source='joined baseboards',motion='fixed'))
for p in meta:
 t=mesh(P[p['id']]);p['watertight']=bool(t.is_watertight);p['solids']=len(t.split());p['bounds']=t.bounds.tolist();t.export(O/(p['id']+'.stl'))
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
# Native LEGO pins, with all six added joints represented.
hm=json.loads((C/'hardware.json').read_text());v=np.load(C/'hardware.npz')['vertices'];arr=[v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in hm]
ph=next(p for p in hm if p['id'].startswith('Memory — Baseboard pin'));pin=v[ph['offset']//3:ph['offset']//3+ph['vertices']].copy();pin-=(pin.min(0)+pin.max(0))/2
newpins=[]
for x in [-30,30]:newpins.append((f'Latch frame mount {x}',[x,38.6,3.],1,'fixed'))
for x in [-30,30]:newpins.append((f'Latch guide joining {x}',[x,46,23.3],2,'fixed'))
for y in [30.4,42.4]:newpins.append((f'Keeper attachment {y}',[15.,y,20.7],2,'carriage'))
for n,c,axis,mo in newpins:
 a=pin.copy()
 if axis==2:a=a@np.array([[1,0,0],[0,0,-1],[0,1,0]]).T
 a+=c;hm.append(dict(id=n,bank='Memory',source='new',motion=mo,lego_part='2780',offset=sum(x.size for x in arr),vertices=len(a)));arr.append(a)
(O/'hardware.json').write_text(json.dumps(hm,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr))
# Exact solid intersections, excluding only the deliberately touching, non-overlapping interfaces.
fixed={p['id']:P[p['id']] for p in meta if p['motion']=='fixed'};cars={p['id']:P[p['id']] for p in meta if p['bank']=='Memory' and p['motion']=='carriage'}
def overlap(a,b):return max(0.,float((a^b).volume()))
fixed_hits=[]
for (na,a),(nb,b) in itertools.combinations(fixed.items(),2):
 vol=overlap(a,b)
 if vol>.005:fixed_hits.append(dict(a=na,b=nb,volume_mm3=vol))
moving_hits=[]
for q in np.linspace(-4.6,4.6,93):
 for n,a in cars.items():
  for nn,b in fixed.items():
   vol=overlap(a.translate([float(q),0,0]),b)
   if vol>.005:moving_hits.append(dict(q=float(q),a=n,b=nn,volume_mm3=vol))
 # Bolt is fully withdrawn while the data carriage travels.
 for n,a in cars.items():
  vol=overlap(a.translate([float(q),0,0]),bolt.translate([0,0,-10]))
  if vol>.005:moving_hits.append(dict(q=float(q),a=n,b='unlocked bolt',volume_mm3=vol))
bolt_hits=[]
for s in np.linspace(-10,0,101):
 b=bolt.translate([0,0,float(s)])
 for n,a in fixed.items():
  vol=overlap(a,b)
  if vol>.005:bolt_hits.append(dict(s=float(s),a='bolt',b=n,volume_mm3=vol))
 for q in [-4.3,4.3]:
  for n,a in cars.items():
   vol=overlap(a.translate([q,0,0]),b)
   if vol>.005:bolt_hits.append(dict(s=float(s),q=q,a='bolt',b=n,volume_mm3=vol))
report=dict(status='INTEGRATION DEVELOPMENT — read all failures before printing',fixed_interferences=fixed_hits,moving_interferences=moving_hits,bolt_interferences=bolt_hits,solidity=[{'id':p['id'],'watertight':p['watertight'],'solids':p['solids']} for p in meta],keeper_positions=93,bolt_positions=101,unresolved=['Write cam and compliant lost-motion drive not installed.','New pin fit and hardware contact checks pending.','No load qualification.','New parts print orientations not yet qualified.'])
(O/'Integration checks.json').write_text(json.dumps(report,indent=2));print('Checks:',len(fixed_hits),len(moving_hits),len(bolt_hits));print('First hits',fixed_hits[:3],moving_hits[:3],bolt_hits[:3]);print('Solids',[(p['id'],p['solids']) for p in meta if p['solids']!=1])
