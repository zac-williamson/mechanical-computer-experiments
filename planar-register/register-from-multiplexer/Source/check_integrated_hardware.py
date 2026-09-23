from pathlib import Path
exec((Path(__file__).parent/'check_integrated_travel.py').read_text().split('hits={}')[0])
hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];H={h['id']:np.unique(v[h['offset']//3:h['offset']//3+h['vertices']],axis=0) for h in hs}
rows={};seen=set()
for qm,qe in itertools.product(np.linspace(-3.75,3.75,9),repeat=2):
 up=max(lift(qe),3.2 if abs(qm)<3.0 else 0)
 for p in meta:
  if p['motion']=='rocker':continue
  off=np.array([qm if p['bank']=='Memory' else qe,0,0]) if p['motion']=='carriage' else np.array([0,0,up]) if p['motion']=='bolt' else np.zeros(3)
  for h in hs:
   q=qm if h['bank']=='Memory' else qe;mo=h['motion'];hoff=np.array([q,0,0]) if mo in ['carriage','worm'] else np.array([0,0,up]) if mo=='bolt' else np.array([np.sign(q)*max(abs(q)-.4,0),0,0]) if mo=='clutch-ring' else np.zeros(3)
   key=(p['id'],h['id'],*np.round(hoff-off,6))
   if key in seen:continue
   seen.add(key);pts=H[h['id']]+hoff-off;t=ts[p['id']];bb=t.bounds;inside=np.all((pts>bb[0]+1e-5)&(pts<bb[1]-1e-5),axis=1);pts=pts[inside]
   if not len(pts):continue
   dist=trimesh.proximity.signed_distance(t,pts);assert np.isfinite(dist).all();dep=float(dist.max());pair=p['id']+' / '+h['id']
   if dep>.08 and (pair not in rows or dep>rows[pair]['depth']):rows[pair]=dict(depth=dep,qm=qm,qe=qe)
print(json.dumps(rows,indent=2));(O/'Development hardware intersections.json').write_text(json.dumps(rows,indent=2))
