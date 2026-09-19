from contact import *
c=np.load(DATA/'tolerance_free.npz');betas=c['betas'];gammas=c['gammas'];ls=np.array([affinity.rotate(pads,b,origin=tuple(pv)) for b in betas]);rows=[]
for dx in [-.25,0,.25]:
 for dy in [-.25,0,.25]:
  for free,off in zip(c['free'],c['offsets']):
   for q,d,expect in [(-4.35,-1,True),(4.325,1,True),(0,-1,False),(0,1,False),(-4.35,1,False),(4.325,-1,False)]:
    valid=free&(area(intersection(ls,affinity.translate(st,q-dx,-dy)))<.0001);r=trace(valid,d,betas,gammas);rows.append(dict(stop_offset=[dx,dy],gear_offset=off.tolist(),q=q,d=d,pass_check=r==expect))
fails=[r for r in rows if not r['pass_check']];print('tolerance checks',len(rows),'fails',len(fails),fails[:5],flush=True)
assert not fails, fails[:10]
