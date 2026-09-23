"""Reject actual viewer transitions with reaction-gear surfaces inside printed levers.
This is a targeted failure test, not an exhaustive collision-free certificate.
"""
from pathlib import Path
import json,re,gzip,base64,hashlib,numpy as np,trimesh
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development'
s=(O/'Viewer.html').read_text();D=json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',s,re.S).group(1));v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);ps={p['id']:p for p in D['parts']}
def mesh(n):
 p=ps[n];a=v[p['offset']//3:p['offset']//3+p['vertices']];return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
lever=mesh('Memory — Short lever');assert lever.is_watertight
gear=mesh('Memory — U022');tri=gear.triangles;pts=np.unique(np.concatenate([gear.vertices,tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]),axis=0)
cache={}
def check(b):
 b=round(b,6)
 if b not in cache:
  T=trimesh.transformations.rotation_matrix(np.radians(-b),[0,1,0],[13.192323604,10.2,32.128448698]);q=trimesh.transform_points(pts,T);lo,hi=lever.bounds;q=q[np.all((q>lo)&(q<hi),axis=1)]
  inside=lever.contains(q);q=q[inside]
  if len(q):
   near,dist,_=trimesh.proximity.closest_point(lever,q);k=int(np.argmax(dist));point=trimesh.transform_points(q[k:k+1],np.linalg.inv(T))[0];cache[b]=dict(depth_mm=float(dist[k]),gear_surface_point=point.tolist(),inside_samples=len(q))
  else:cache[b]=dict(depth_mm=0,inside_samples=0)
 return cache[b]
rows=[]
for c in D['auditCases']:
 hits=[]
 for i,f in enumerate(c['frames']):
  for bank,key in [('Memory','bm'),('Write','be')]:
   r=check(f[key])
   if r['depth_mm']>.03:hits.append(dict(frame=i,bank=bank,angle_deg=f[key],**r))
 worst=max(hits,key=lambda r:r['depth_mm']) if hits else None
 rows.append(dict(initial=c['initial'],target=c['target'],profile=c.get('profile'),frames=len(c['frames']),failed_frames=len(set(r['frame'] for r in hits)),worst=worst,status='FAIL' if hits else 'NO TARGETED HIT'))
r=dict(status='FAIL' if any(x['worst'] for x in rows) else 'INCOMPLETE',viewer_sha256=hashlib.sha256(s.encode()).hexdigest(),parts=len(D['parts']),frames=sum(x['frames'] for x in rows),cases=len(rows),input_transitions=len(set(tuple(c['initial'][:2]+c['target']) for c in rows)),method='Closed printed lever: native reaction gear vertices, edge midpoints and face centres tested inside the lever at the ACTUAL displayed angle. No intended-mesh exemption. 0.03 mm reporting threshold.',scope='Targeted actual-render failure check across every exported timing frame. Does not assert all other pairs clear; full all-pair material scan is separately diagnostic.',cases_detail=rows,angles={str(k):val for k,val in cache.items()})
(O/'Actual viewer transition failures.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k not in ['cases_detail','angles']},indent=2));print('Worst',max((x['worst'] for x in rows if x['worst']),key=lambda x:x['depth_mm']))
raise SystemExit(1 if r['status']=='FAIL' else 2)
