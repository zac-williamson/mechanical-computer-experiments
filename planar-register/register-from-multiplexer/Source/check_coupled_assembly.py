"""All-pair candidate and closed-printed-solid audit using actual coupled poses.
No assembly-group exemptions. Native/native surface contacts remain classified separately.
"""
from pathlib import Path
import json,re,base64,gzip,numpy as np,trimesh,manifold3d as m
from coupled_pose import pose
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';s=(O/'Viewer.html').read_text();D=json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',s,re.S).group(1));v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);ps=D['parts'];M={p['id']:p for p in ps};P={};S={};pts={};cm=trimesh.collision.CollisionManager()
for p in ps:
 a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);t.merge_vertices(digits_vertex=5);t.update_faces(t.nondegenerate_faces(height=1e-8));t.update_faces(t.unique_faces());
 if p['kind'] in ['printed','structure']:t=trimesh.load(O/(p['id']+'.stl'))
 P[p['id']]=t;cm.add_object(p['id'],t)
 if p['kind'] in ['printed','structure']:
  assert t.is_watertight,p['id'];S[p['id']]=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 else:
  tri=t.triangles;pts[p['id']]=np.unique(np.concatenate([t.vertices,tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]),axis=0)
hits={};native={};seen=set();count=0;seen_frames=set();bandup=None
for ci,c in enumerate(D['auditCases']):
 # Every exported frame is included. Repeated complete states are reused.
 for fi,f in enumerate(c['frames']):
  key=tuple(round(f[k]%(360 if k in ['wm','we','gm','ge','power','data','output','roller'] else 1e9),5) for k in ['qm','qe','rm','re','s','bm','be','wm','we','gm','ge','power','data','output','roller'])
  if key in seen_frames:continue
  seen_frames.add(key);count+=1
  if bandup!=f['s']:
   bandup=f['s'];t=P['Lock elastic band'].copy();a=t.vertices.copy();a[a[:,2]>58.5,2]+=bandup;t.vertices=a;cm.remove_object('Lock elastic band');cm.add_object('Lock elastic band',t);tri=t.triangles;pts['Lock elastic band']=np.unique(np.concatenate([t.vertices,tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]),axis=0)
  T={p['id']:pose(p,f) for p in ps}
  for n,t in T.items():cm.set_transform(n,t)
  _,pairs=cm.in_collision_internal(return_names=True)
  for a,b in pairs:
   if a not in S and b in S:a,b=b,a
   rel=np.linalg.inv(T[a])@T[b];pk=(a,b,tuple(np.round(rel[:3,:].ravel(),5)),round(f['s'],5) if 'Lock elastic band' in [a,b] else 0)
   if pk in seen:continue
   seen.add(pk)
   if a not in S:
    native.setdefault(tuple(sorted((a,b))),dict(pair=sorted((a,b)),case=ci,frame=fi));continue
   if b in S:
    vol=max(0,float((S[a]^S[b].transform(rel[:3,:])).volume()));value=vol;kind='solid_overlap_mm3';threshold=.005
   else:
    pp=trimesh.transform_points(pts[b],rel);bounds=P[a].bounds;pp=pp[np.all((pp>bounds[0])&(pp<bounds[1]),axis=1)];pp=pp[P[a].contains(pp)] if len(pp) else pp;value=float(trimesh.proximity.closest_point(P[a],pp)[1].max()) if len(pp) else 0.;kind='native_surface_inside_print_mm';threshold=.03
   pair=(a,b)
   if value>threshold and value>hits.get(pair,{}).get('value',0):hits[pair]=dict(pair=[a,b],value=value,measurement=kind,case=ci,frame=fi,qm=f['qm'],qe=f['qe']);print(json.dumps(hits[pair]),flush=True)
 if ci%6==0:print('CASES',ci+1,'unique frames',count,'narrow pairs',len(seen),flush=True)
r=dict(parts=len(ps),cases=len(D['auditCases']),exported_frames=sum(len(c['frames']) for c in D['auditCases']),unique_frames=count,all_part_group_exclusions=[],printed_or_printed_native_hits=list(hits.values()),native_native_contacts=list(native.values()),limits='Finite displayed frames and finite native surface points. Friction pin fits are reported, not automatically exempted. Native/native intersections are candidates, not valid signed depths for open LEGO surfaces. No load or continuous-sweep certification.')
(O/'Coupled assembly contacts.json').write_text(json.dumps(r,indent=2));print('COMPLETE',count,'frames',len(hits),'printed/native hit pairs',len(native),'native/native candidates',flush=True)
