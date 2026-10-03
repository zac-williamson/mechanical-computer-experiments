"""Fixed inherited phases, actual gear adjacency and independently keyed ports."""
from pathlib import Path
import json,hashlib,copy,math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
from wall_pose import joint
R=Path(__file__).resolve().parents[1];O=R/'Functional register'
P=json.load(open(O/'parts.json'));V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
f=dict(rail=0,rm=0,ro=0,rw=0,master_lift=4,slave_lift=4,angles={k:0. for k in ['D','WRITE','CLK','POWER','X','M','Q']})
for b in ['master','slave','write','clock']:f[b]=dict(q=0,g=0,b=0,w=0)
items=[]
for p in P:
 n=p['id'];N={'94925':16,'3648':24,'10928':8}.get(p.get('lego_part'))
 if n.endswith((' L072',' L102',' B-input')) or n=='slave data port gear':N=16
 if N is None:continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']];c=(a.min(0)+a.max(0))/2
 rad=np.linalg.norm(a[:,1:]-c[1:],axis=1);face=a[rad>N/2+.15,0]
 rates={}
 for d in f['angles']:
  h=copy.deepcopy(f);h['angles'][d]=1;rates[d]=float(joint(p,h)[3]-joint(p,f)[3])
 tri=a.reshape(-1,3,3)[:,:,1:]-c[1:];u=tri[:,1]-tri[:,0];w=tri[:,2]-tri[:,0];area=abs(u[:,0]*w[:,1]-u[:,1]*w[:,0])
 profile=unary_union([Polygon(t) for t in tri[area>1e-8]])
 items.append(dict(p=p,N=N,centre=c,face=[face.min(),face.max()],rates=rates,profile=profile,phase=math.degrees(joint(p,f)[3])))
expected=[]
for bank in ['master','slave']:
 expected += [[bank+' L072',bank+(' lower' if bank=='master' else ' upper')+' clutch supply'],[bank+' L102',bank+(' upper' if bank=='master' else ' lower')+' clutch supply'],[bank+' inverter lower',bank+' inverter upper'],[bank+' inverter lower',bank+' power input gear']]
expected += [['Selected data route 16T','master_gate B-input'],['master_gate B-input','master_gate L102'],['slave data port gear','slave_gate L102']]
edges=[]
for i,a in enumerate(items):
 for b in items[i+1:]:
  delta=b['centre'][1:]-a['centre'][1:];distance=np.linalg.norm(delta);overlap=min(a['face'][1],b['face'][1])-max(a['face'][0],b['face'][0])
  if overlap<.1 or abs(distance-(a['N']+b['N'])/2)>.05:continue
  errors={d:abs(b['rates'][d]+a['N']/b['N']*a['rates'][d]) for d in a['rates']}
  theta=np.linspace(0,360/a['N'],97)
  peak=max(affinity.rotate(a['profile'],float(t+a['phase']),origin=(0,0)).intersection(affinity.translate(affinity.rotate(b['profile'],float(-t*a['N']/b['N']+b['phase']),origin=(0,0)),*delta)).area for t in theta)
  row=dict(a=a['p']['id'],b=b['p']['id'],teeth=[a['N'],b['N']],centre_distance_mm=float(distance),ratio=-a['N']/b['N'],max_velocity_error_rad=max(errors.values()),fixed_phase_a_deg=a['phase'],fixed_phase_b_deg=b['phase'],max_projected_tooth_overlap_mm2=float(peak));edges.append(row)
  print(row['a'],row['b'],'phase overlap',round(peak,6),flush=True)
match=sorted(sorted(x) for x in expected)==sorted(sorted([e['a'],e['b']]) for e in edges)
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts_sha256=hashlib.sha256((O/'parts.json').read_bytes()).hexdigest(),meshes=edges,expected_meshes=expected,topology_pass=match,unit_magnitude_pass=match and all(abs(e['ratio'])==1 for e in edges),rotation_identity_pass=match and all(e['max_velocity_error_rad']<1e-9 for e in edges),fixed_external_phase_pass=match and all(e['max_projected_tooth_overlap_mm2']<1e-8 for e in edges),limits=['External tooth profiles and speed identities only; inherited clutch/lock contact traces do not prove new loaded dynamics.','Steady-state RPM magnitude; reversal and dog-clutch transitions necessarily include transients.'])
(O/'Transmission checks.json').write_text(json.dumps(report,indent=2))
print('TOPOLOGY',match,'PHASE',report['fixed_external_phase_pass'],flush=True)
