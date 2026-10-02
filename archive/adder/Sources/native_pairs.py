exec(open(__file__.replace('native_pairs.py','native_audit.py')).read().split('fixed={}')[0])
records=[r for r in D['records'] if r['part'] not in ['2780.dat','32905.dat','18947.dat','26287.dat']]
sol={};axes={};gearparts=['4019.dat','10928.dat','18946.dat'];axleparts=['3705.dat','3706.dat','3707.dat','3708.dat','3737.dat','60485.dat','23948.dat','4519.dat','32073.dat','44294.dat']
for r in records:
 v,_=lib.mesh(r['part']);v=(v*.4@np.array(r['matrix']).reshape(3,3).T+np.array(r['pos'])*.4).reshape(-1,3)
 sol[r['record_id']]=solid(trimesh.convex.convex_hull(v))
 axes[r['record_id']]=np.array(r['matrix']).reshape(3,3)@([1,0,0] if r['part'] in axleparts else [0,0,1])
intended={frozenset(p) for p in D['gear_pairs']};hits=[]
for i,r in enumerate(records):
 n=r['record_id'];a=sol[n];aa=np.array(a.bounding_box());pa=np.array(r['pos'])*.4
 for s in records[i+1:]:
  k=s['record_id'];b=sol[k];bb=np.array(b.bounding_box());pb=np.array(s['pos'])*.4
  if frozenset([n,k]) in intended:continue
  if np.any(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])<.001):continue
  sameaxis=abs(np.dot(axes[n],axes[k]))>.999 and np.linalg.norm(np.cross(pa-pb,axes[n]))<.001
  if sameaxis and ((r['part'] in axleparts) != (s['part'] in axleparts)):continue
  vol=(a^b).volume()
  if vol>.1:hits.append([n,k,round(vol,3)])
(O/'Native pair checks.json').write_text(json.dumps(dict(method='Conservative hulls, excluding intended gear meshes and coaxial gear/bush engagement; overlapping axles are retained as errors.',hits=hits),indent=2))
print('Native pair hits',len(hits));print(json.dumps(hits,indent=2))
