"""Discover actual gear adjacencies and check angular speed identities."""
from pathlib import Path
import json,hashlib,copy
import numpy as np
from wall_pose import joint
O=Path(__file__).resolve().parents[1]/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
f=dict(rail=0,rm=0,ro=0,rw=0,master_lift=4,slave_lift=4,angles={k:0. for k in ['D','WRITE','CLK','POWER','X','M','Q']})
for b in ['master','slave','write','clock']:f[b]=dict(q=0,g=0,b=0,w=0)
items=[]
for p in P:
 n=p['id'];N={'94925':16,'3648':24,'10928':8}.get(p.get('lego_part'))
 if n.endswith((' L072',' L102',' B-input')):N=16
 if N is None:continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']]
 if 'assembly_rotation' in p:a=(a-np.array(p['assembly_translation']))@np.array(p['assembly_rotation'])
 c=(a.min(0)+a.max(0))/2;r=np.linalg.norm(a[:,1:]-c[1:],axis=1);face=a[r>N/2+.15,0]
 rates={}
 for d in f['angles']:
  h=copy.deepcopy(f);h['angles'][d]=1
  if d=='CLK':h['clock']['w']=1
  if d=='WRITE':h['write']['w']=-1
  rates[d]=float(joint(p,h)[3]-joint(p,f)[3])
 items.append((p,N,c,[face.min(),face.max()],rates))
edges=[]
for i,(p,n,c,face,ra) in enumerate(items):
 for q,nn,cc,ff,rb in items[i+1:]:
  if p.get('mesh_domain','bit')!=q.get('mesh_domain','bit'):continue
  dist=float(np.linalg.norm(c[1:]-cc[1:]));overlap=float(min(face[1],ff[1])-max(face[0],ff[0]))
  if overlap<.1 or abs(dist-(n+nn)/2)>.05:continue
  errors={d:abs(rb[d]+n/nn*ra[d]) for d in ra}
  edges.append(dict(a=p['id'],b=q['id'],teeth=[n,nn],ratio=-n/nn,centre_distance_mm=dist,face_overlap_mm=overlap,max_velocity_error_rad=max(errors.values())))
expected_pairs=[['master L072', 'master POWER 16T 16'], ['master L102', 'master reversing idler 16T -16'], ['master POWER 16T 32', 'master reversing idler 16T 32'], ['POWER header gear', 'master POWER 16T 32'], ['slave L072', 'slave reversing idler 16T 16'], ['slave L102', 'slave POWER 16T -16'], ['master_gate B-input', 'master_gate L102'], ['Selected data route 16T', 'master_gate B-input'], ['slave_gate B-input', 'slave_gate L102'], ['Q feedback front gear -92', 'write L072'], ['WRITE D input gear', 'write L102'], ['D header input 8T', 'D header output 8T'], ['Q feedback front gear 150', 'Q takeoff gear'], ['Control CLK input 16T', 'Control CLK receiving 16T']]
actual_pairs=sorted(sorted([e['a'],e['b']]) for e in edges)
graph_matches=actual_pairs==sorted(expected_pairs)
out=dict(expected_mesh_pairs=expected_pairs,topology_pass=graph_matches,geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),meshes=edges,unit_magnitude_pass=graph_matches and all(abs(e['ratio'])==1 for e in edges),rotation_identity_pass=graph_matches and all(e['max_velocity_error_rad']<1e-9 for e in edges),scope='Actual CAD-discovered external mesh identities for all seven independent angle drivers; excludes engagement dynamics',constant_rpm_during_reversal=False)
(O/'Rotation ratio checks.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
