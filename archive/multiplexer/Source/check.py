from geometry import *
from trimesh.collision import CollisionManager
import manifold3d as m,itertools
params=json.loads((OUT/'parameters.json').read_text());SY=params['worm_center'][1];G=params['worm_center'][2];GY=params['reaction_center'][1]
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
meta=json.loads((OUT/'printed-parts.json').read_text());parts={p['id']:trimesh.load(OUT/(p['id']+'.stl')) for p in meta};solids={n:solid(t) for n,t in parts.items()}
pv=np.array(params['pivot'])
fixed={p['id']:solids[p['id']] for p in meta if p['motion']=='fixed'};cars={p['id']:solids[p['id']] for p in meta if p['motion']=='carriage'}
report={'fixed_overlaps':[],'moving_overlaps':[],'solidity':[{k:p[k] for k in ['id','solids','watertight']} for p in meta]}
for (a,s),(b,t) in itertools.combinations(fixed.items(),2):
 v=(s^t).volume()
 if v>1e-3:report['fixed_overlaps'].append(dict(a=a,b=b,volume=v))
for q in np.linspace(-4.6,4.575,185):
 for a,s in cars.items():
  for b,t in fixed.items():
   v=(s.translate([q,0,0])^t).volume()
   if v>1e-3:report['moving_overlaps'].append(dict(a=a,b=b,q=float(q),volume=v))
rows=json.loads((OUT/'Switching trace.json').read_text())['frames'];seen=set();lh=[]
for r in rows:
 q,beta=r['q'],r['b'];key=(round(q,2),round(beta,1))
 if key in seen:continue
 seen.add(key)
 if len(seen)%1:continue
 s=solids['Short lever'].translate(-pv).rotate([0,beta,0]).translate(pv)
 for b,t in {**fixed,**{n:t.translate([q,0,0]) for n,t in cars.items()}}.items():
  v=(s^t).volume()
  if v>.005:lh.append(dict(b=b,q=q,beta=beta,volume=v))
report['lever_overlaps']=lh
# Native LEGO mesh against prints at selected trace poses; broad-phase/contact inventory.
hm=json.loads((OUT/'hardware.json').read_text());vv=np.load(OUT/'hardware.npz')['vertices'];hw={p['id']:trimesh.Trimesh(vv[p['offset']//3:p['offset']//3+p['vertices']],np.arange(p['vertices']).reshape(-1,3),process=False) for p in hm}
cm=CollisionManager()
for n,t in parts.items():cm.add_object(n,t)
cmh=CollisionManager()
for n,t in hw.items():cmh.add_object(n,t)
hits={}
for r in rows[::max(1,len(rows)//180)]:
 q,b,w,g=r['q'],r['b'],r['w'],r['g']
 for p in meta:
  T=np.eye(4)
  if p['motion']=='carriage':T[0,3]=q
  elif p['motion']=='rocker':T=trimesh.transformations.rotation_matrix(np.radians(b),[0,1,0],pv)
  cm.set_transform(p['id'],T)
 for p in hm:
  n=p['id'];mo=p.get('motion');T=np.eye(4)
  if mo=='gear':T=trimesh.transformations.rotation_matrix(np.radians(g),[0,1,0],params['reaction_center'])
  elif mo in ['input','worm']:
   T=trimesh.transformations.rotation_matrix(np.radians(w),[1,0,0],[0,SY,G])
   if mo=='worm':T[0,3]=q
  elif mo=='carriage':T[0,3]=q
  elif mo=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
  cmh.set_transform(n,T)
 yes,pairs=cm.in_collision_other(cmh,return_names=True)
 for pair in pairs:
  k=' | '.join(pair);hits[k]=hits.get(k,0)+1
report['native_print_hardware_contacts']=hits
# Only specific designed interfaces may contact; no blanket hardware exemptions.
allowed={('Carriage fork and roof','L099'),('Short lever','U022')}
for h in hm:
 if h['id'].startswith('Cartridge'):
  allowed.update((n,h['id']) for n in ['Right side frame','Front bearing cheek','Rear bearing cheek'])
for h in hm:
 if h['id'].startswith('Carriage support pin'):
  allowed.update((n,h['id']) for n in ['Carriage fork and roof','Left carriage bearing support' if 'pin -1 ' in h['id'] else 'Right carriage bearing support'])
mounts=json.loads((OUT/'baseboard-mounts.json').read_text())
for mount in mounts:
 pin=f"Baseboard pin X{mount['x']} Z{mount['z']}"
 allowed.update((n,pin) for n in ['Common baseboard',mount['wall']])
report['unexpected_native_print_contacts']={k:v for k,v in hits.items() if tuple(k.split(' | ')) not in allowed}
bs=[t.bounds.copy() for t in parts.values()]+[t.bounds.copy() for t in hw.values()]
for n,t in parts.items():
 if n in cars:
  b=t.bounds.copy();b[0,0]-=4.6;b[1,0]+=4.575;bs.append(b)
lo=np.min(bs,axis=0)[0];hi=np.max(bs,axis=0)[1];ext=hi-lo
report['envelope']={'min':lo.tolist(),'max':hi.tolist(),'size':ext.tolist(),'volume_mm3':float(np.prod(ext)),'reduction_vs_baseline':float(1-np.prod(ext)/(96*60.2*116)),'band_and_lever_sweep_not_yet_included':True}
(OUT/'checks.json').write_text(json.dumps(report,indent=2))
for k,v in report.items():
 if isinstance(v,list): print(k,len(v),v[:8])
 else:print(k,v)
items=[]
for p in meta:items.append((dict(id=p['id'],color=[45,140,154] if p['motion']=='carriage' else [229,171,45] if p['motion']=='rocker' else [99,129,119]),parts[p['id']].triangles.reshape(-1,3)))
items.extend((p,hw[p['id']].vertices) for p in hm)
plot(items,OUT/'orthographic.png')
