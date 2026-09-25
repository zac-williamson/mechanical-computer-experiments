"""Check bearing lands, axle engagement, stops and shared-drive connections."""
from pathlib import Path
import json,hashlib
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);lookup={p['id']:p for p in P}
schedule=json.loads((O/'Bearing schedule.json').read_text())
def mesh(p):return V[p['offset']//3:p['offset']//3+p['vertices']]
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def cyl(r,lo,hi,axis,c):
 s=m.Manifold.cylinder(hi-lo,r,circular_segments=32)
 if axis==0:s=s.rotate([0,90,0])
 elif axis==1:s=s.rotate([-90,0,0])
 c=np.array(c,dtype=float).copy();c[axis]=lo;return s.translate(c)
fixed={mod:sum((solid(mesh(p)) for p in P if p['module']==mod and p['kind']=='printed' and p.get('motion','fixed')=='fixed'),m.Manifold()) for mod in ['bit','control']}
results=[]
for b in schedule['bearings']:
 if b['status']!='complete bearing generated':results.append(dict(**b,pass_check=False));continue
 axis='XYZ'.index(b['axis']);t=b['station_mm'];half=b['land_mm']/2
 arrays=[mesh(lookup[n]) for n in b['axle_parts']];a=np.concatenate(arrays);c=(a.min(0)+a.max(0))/2
 ring=cyl(5,t-half,t+half,axis,c)-cyl(2.65,t-half-.1,t+half+.1,axis,c)
 missing=(ring-fixed[b['module']]).volume()
 occupied=(cyl(2.59,t-half,t+half,axis,c)^fixed[b['module']]).volume()
 engagement=max(max(0,min(t+half,a[:,axis].max())-max(t-half,a[:,axis].min())) for a in arrays)
 results.append(dict(shaft=b['shaft'],station_mm=t,land_mm=b['land_mm'],missing_ring_mm3=missing,occupied_bore_mm3=occupied,axle_engagement_mm=engagement,pass_check=missing<.001 and occupied<.001 and engagement>=b['land_mm']-.001))
# The one shared reversing connection is a physical keyed coupler, not an animation link.
parts=[lookup[n] for n in ['Shared reverse shaft left 10L','Shared reverse coupling','Shared reverse shaft right 10L']]
bounds=[np.array(p['bounds']) for p in parts];coupler=bounds[1]
engagements=[float(min(b[1,0],coupler[1,0])-max(b[0,0],coupler[0,0])) for b in [bounds[0],bounds[2]]]
centres=[b.mean(0)[1:] for b in bounds];aligned=all(np.linalg.norm(c-centres[0])<1e-6 for c in centres)
retained=all(x['status']=='opposed axial stops assigned' for x in schedule['retention'])
out=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,bearings=results,retention=schedule['retention'],shared_reverse_coupling=dict(axially_aligned=aligned,engagement_each_end_mm=engagements,pass_check=aligned and min(engagements)>=7),support_layout_pass=all(x['pass_check'] for x in results) and retained and aligned and min(engagements)>=7,limitations=['Axial-stop grip and coupler grip depend on physical part fit; no torque/force rating.', 'Checks explicit transmission bearings. Retained actuator pivots and moving carriage pins require physical fit checks.', 'A solid bearing wall is not a printed-strength calculation.'],mechanically_qualified=False)
(O/'Bearing and retention checks.json').write_text(json.dumps(out,indent=2));print('support layout',out['support_layout_pass'],'bearings',len(results),'retained assemblies',len(schedule['retention']))
for r in results:
 if not r['pass_check']:print(r)
