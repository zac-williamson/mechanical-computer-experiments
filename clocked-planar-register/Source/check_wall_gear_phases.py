"""Phase screen for each CAD-discovered external mesh, with inherited phases fixed.
New idler shafts may be assembled at a free phase; inherited shafts may not.
"""
from pathlib import Path
import json,hashlib,math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
from wall_pose import joint
O=Path(__file__).resolve().parents[1]/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
ratios=json.loads((O/'Rotation ratio checks.json').read_text())
assert ratios['geometry_sha256']==hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(), 'Stale ratio report'
assert ratios['rotation_identity_pass'] and ratios['unit_magnitude_pass'], 'Invalid gear routes'
f=dict(rail=0,rm=0,ro=0,rw=0,master_lift=4,slave_lift=4,angles={k:0. for k in ['D','WRITE','CLK','POWER','X','M','Q']})
for b in ['master','slave','write','clock']:f[b]=dict(q=0,g=0,b=0,w=0)
oldph=json.loads((O.parent/'Compact layout/External gear phase checks.json').read_text())['shaft_phases_deg']

from wall_phases import group
ph={k:oldph[k] for k in ['Master output','POWER internal','Q inverted']};ph['CLK worm']=0
items={}
for p in P:
 n=p['id'];N={'94925':16,'3648':24,'10928':8}.get(p.get('lego_part'))
 if n.endswith((' L072',' L102',' B-input')):N=16
 if N is None:continue
 a=V[p['offset']//3:p['offset']//3+p['vertices']]
 if 'assembly_rotation' in p:a=(a-np.array(p['assembly_translation']))@np.array(p['assembly_rotation'])
 c=(a.min(0)+a.max(0))/2
 tri=a.reshape(-1,3,3)[:,:,1:]-c[1:];u=tri[:,1]-tri[:,0];w=tri[:,2]-tri[:,0];ar=abs(u[:,0]*w[:,1]-u[:,1]*w[:,0])
 profile=unary_union([Polygon(t) for t in tri[ar>1e-8]])
 g=group(p)
 if 'baseline_id' in p:ph[g]=math.degrees(joint(p,f)[3])
 items[n]=dict(profile=profile,centre=c,N=N,group=g)
edges=ratios['meshes'].copy();out=[]
while edges:
 found=next(((i,e) for i,e in enumerate(edges) if items[e['a']]['group'] in ph or items[e['b']]['group'] in ph),None)
 if found is None:ph[items[edges[0]['a']]['group']]=0;continue
 i,e=found;edges.pop(i);a,b=items[e['a']],items[e['b']]
 if a['group'] not in ph:a,b=b,a
 pa=ph[a['group']];delta=b['centre'][1:]-a['centre'][1:];theta=np.linspace(0,360/a['N'],49)
 rotated=[affinity.rotate(a['profile'],float(t+pa),origin=(0,0)) for t in theta]
 def maximum(pb):return max(aa.intersection(affinity.translate(affinity.rotate(b['profile'],float(-t*a['N']/b['N']+pb),origin=(0,0)),*delta)).area for t,aa in zip(theta,rotated))
 if b['group'] not in ph:
  best=min((maximum(float(pb)),float(pb)) for pb in np.arange(0,360/b['N'],.125));ph[b['group']]=best[1]
 overlap=maximum(ph[b['group']]);out.append(dict(a=e['a'],b=e['b'],max_projected_overlap_mm2=overlap))
 print(e['a'],e['b'],round(overlap,6),flush=True)
result=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,shaft_phases_deg=ph,meshes=out,external_profile_pass=all(e['max_projected_overlap_mm2']<1e-8 for e in out),mechanically_qualified=False)
(O/'External gear phase checks.json').write_text(json.dumps(result,indent=2))
