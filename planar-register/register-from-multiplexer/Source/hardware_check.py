from pathlib import Path
import json,numpy as np,trimesh
from trimesh.collision import CollisionManager
R=Path(__file__).resolve().parents[1];O=R/'Assembly';meta=json.loads((O/'printed-parts.json').read_text());hm=json.loads((O/'hardware.json').read_text());vv=np.load(O/'hardware.npz')['vertices']
cp=CollisionManager();ch=CollisionManager()
for p in meta:cp.add_object(p['id'],trimesh.load(O/(p['id']+'.stl')))
for h in hm:
 a=vv[h['offset']//3:h['offset']//3+h['vertices']];ch.add_object(h['id'],trimesh.Trimesh(a,np.arange(h['vertices']).reshape(-1,3),process=False))
changed={'Common two-core baseboard','Memory — Right carriage bearing support','Memory — Keeper and rear arm','Memory — Fixed lower latch guide','Memory — Removable upper latch guide','Memory — Lock bolt'}|{'Memory — '+x for x in ['Left side frame','Right side frame','Left inner bearing wall','Right inner bearing wall']}
new_hardware={h['id'] for h in hm if h.get('source')=='new'}
allowed=set()
for h in hm:
 n=h['id']
 if 'Baseboard pin' in n:
  allowed.add(('Common two-core baseboard',n))
  bank=h['bank'];src=h['source'];x=float(src.split('X')[1].split(' ')[0]);wall=('Left ' if x<0 else 'Right ')+('side frame' if abs(x)==28 else 'inner bearing wall');allowed.add((bank+' — '+wall,n))
 if n.startswith('Keeper attachment'):
  allowed.update((p,n) for p in ['Memory — Right carriage bearing support','Memory — Keeper and rear arm'])
 if n.startswith('Latch frame mount'):
  allowed.update((p,n) for p in ['Common two-core baseboard','Memory — Fixed lower latch guide'])
 if n.startswith('Latch guide joining'):
  allowed.update((p,n) for p in ['Memory — Fixed lower latch guide','Memory — Removable upper latch guide'])
 if n.startswith('Memory — Carriage support pin'):
  allowed.add(('Memory — Right carriage bearing support',n))
 if n.startswith('Memory — Cartridge'):
  allowed.add(('Memory — Right side frame',n))
seen={};allpairs={}
for q in np.linspace(-4.6,4.6,47):
 for p in meta:
  T=np.eye(4)
  if p['bank']=='Memory' and p['motion']=='carriage':T[0,3]=q
  if p['motion']=='bolt':T[2,3]=-10
  cp.set_transform(p['id'],T)
 for h in hm:
  T=np.eye(4)
  if h['bank']=='Memory':
   if h['motion'] in ['carriage','worm']:T[0,3]=q
   if h['motion']=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
  ch.set_transform(h['id'],T)
 yes,pairs=cp.in_collision_other(ch,return_names=True)
 for a,b in pairs:
  if a not in changed and b not in new_hardware:continue
  allpairs[(a,b)]=allpairs.get((a,b),0)+1
  if (a,b) not in allowed:
   seen.setdefault((a,b),dict(a=a,b=b,first_q=float(q),samples=0));seen[(a,b)]['samples']+=1
report=dict(method='Native triangle meshes against printed meshes, 47 memory positions, bolt withdrawn; source gear angular phases fixed. Contact inventory, not a full phase or penetration-depth proof.',unexpected_contacts=list(seen.values()),allowed_contacts=[dict(printed=a,hardware=b,samples=n) for (a,b),n in allpairs.items() if (a,b) in allowed],qualification='Intentional friction-pin seating allowed only for named mated parts. Locked/closing bolt and all-rotation hardware envelopes still need checking.')
(O/'Hardware contacts.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
