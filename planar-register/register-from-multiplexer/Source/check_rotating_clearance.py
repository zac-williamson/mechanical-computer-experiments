from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register';s=(R/'Source/full_contact_inventory.py').read_text();exec(s[:s.index('records={}')])
cp=trimesh.collision.CollisionManager();ch=trimesh.collision.CollisionManager();P={};H={};M={p['id']:p for p in meta}
for p in ps:P[p['id']]=trimesh.load(O/(p['id']+'.stl'));cp.add_object(p['id'],P[p['id']])
for h in hs:
 if h['motion'] in ['fixed','carriage','release-crank','gear','bolt']:continue
 a=v[h['offset']//3:h['offset']//3+h['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False);H[h['id']]=t;ch.add_object(h['id'],t)
def axis(n):
 if 'joiner' in n:return [0,10.2,16]
 bank=16 if n.startswith('Write') else 0
 if 'idler' in n:return [0,10.2+np.sqrt(33.75),-10.5]
 if 'input' in n or 'power axle' in n or 'B-shaft' in n:
  if 'selector' not in n:return [0,10.2,0 if bank else -16]
 if any(x in n for x in ['C-shaft','U072','selector-right','U015']):return [0,10.2,16+bank]
 return [0,10.2,bank]
ring_envelopes=[]
for bank,z in [('Memory',0),('Write',16)]:
 h=next(h for h in hs if h['id']==bank+' — L099');a=v[h['offset']//3:h['offset']//3+h['vertices']];tri=a.reshape(-1,3,3);xs=np.unique(a[:,0]);env=m.Manifold()
 for l,r in zip(xs[:-1],xs[1:]):
  if r-l<1e-5:continue
  active=tri[(tri[:,:,0].min(1)<r-1e-6)&(tri[:,:,0].max(1)>l+1e-6)]
  if not len(active):continue
  radius=np.linalg.norm(active[:,:,[1,2]]-[10.2,z],axis=2).max()/np.cos(np.pi/256)
  env+=m.Manifold.cylinder(r-l+.8,radius,circular_segments=256).rotate([0,90,0]).translate([l-.4,10.2,z])
 t=trimesh.load(O/(bank+' — Carriage fork and roof.stl'));p=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 for shift in [0]:
  vol=max(0,float((env.translate([shift,0,0])^p).volume()));assert vol<.005,(bank,shift,vol);ring_envelopes.append(dict(bank=bank,relative_axial_sweep_mm=[-.4,.4],conservative_swept_overlap_mm3=vol))

seen=set();hits={};contacts={};count=0
for qm in [-4.6,-2.3,0,2.3,4.6]:
 for qe in [-4.6,-2.3,0,2.3,4.6]:
  T={p['id']:pose(p,qm,qe) for p in ps}
  for n,t in T.items():cp.set_transform(n,t)
  for deg in range(0,360,5):
   count+=1;U={}
   for n in H:
    u=trimesh.transformations.rotation_matrix(np.radians(deg),[1,0,0],axis(n));q=qm if M[n].get('bank')=='Memory' else qe;u[0,3]=q if M[n]['motion']=='worm' else np.sign(q)*max(abs(q)-.4,0) if M[n]['motion']=='clutch-ring' else 0;U[n]=u;ch.set_transform(n,u)
   _,pairs=cp.in_collision_other(ch,return_names=True)
   for a,b in pairs:
    if a.endswith('Carriage fork and roof') and b.endswith('L099') and a.split(' — ')[0]==b.split(' — ')[0]:continue # covered by conservative full-rotation envelope above
    rel=np.linalg.inv(T[a])@U[b];key=(a,b,tuple(np.round(rel[:3].ravel(),5)))
    if key in seen:continue
    seen.add(key);contacts.setdefault((a,b),0);contacts[a,b]+=1
    tri=H[b].triangles;pts=np.concatenate([H[b].vertices,tri.mean(1),(tri[:,0]+tri[:,1])/2,(tri[:,1]+tri[:,2])/2,(tri[:,2]+tri[:,0])/2]);pts=trimesh.transform_points(pts,rel);bo=P[a].bounds;pts=pts[np.all((pts>=bo[0]-.001)&(pts<=bo[1]+.001),axis=1)]
    dep=float(trimesh.proximity.signed_distance(P[a],pts).max()) if len(pts) else 0
    if dep>.03 and dep>hits.get((a,b),{}).get('depth_mm',0):hits[a,b]=dict(pair=[a,b],depth_mm=dep,qm=qm,qe=qe,rotation_deg=deg)
 print('Memory q',qm,'done',flush=True)
r=dict(clutch_ring_full_rotation_envelopes=ring_envelopes,poses=count,rotation_step_deg=5,carriage_positions_per_axis=5,hardware_parts=len(H),penetrations=list(hits.values()),contacts=[dict(pair=k,relative_poses=x) for k,x in contacts.items()],limits='Every modelled X-axis drive part rotated independently for printed clearance. Gear-to-gear phase/engagement and the Y-axis reaction mechanism are separate checks; finite surface samples, rigid motion.');(O/'Rotating clearance checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
