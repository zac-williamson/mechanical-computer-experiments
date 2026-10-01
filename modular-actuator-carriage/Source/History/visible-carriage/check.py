from pathlib import Path
import os,json,math
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];O=R0/'visible-carriage-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
exec(compile(s,'operating checks','exec'));assert not printed_hits and not native_hits and not band_hits
by={p['name']:p['s'] for p in parts}
def load(n):
 t=trimesh.load_mesh(O/'baseline'/(n+'.stl'));return solid(t)
rows={}
old=load('Carriage body')+load('Carriage bearing end');new=by['Carriage body']+by['Carriage bearing end']
regions={'clutch surfaces':cy(7.4,-8,7.8,0,[0,10.2,0]),'lever contact':box([-.307,30,11.601],[26.61,37.49,19.999]),'both locking pockets':box([-12,23,35.7],[3,30,39.5])}
for sign in [-1,1]:
 xa,xb=sorted([sign*9.101,sign*16.249]);regions['worm thrust '+str(sign)]=cy(5.1,xa,xb,0,[0,10.2,16])-cy(3.46,xa-.01,xb+.01,0,[0,10.2,16])
for k,v in regions.items():
 delta=(((old-new)+(new-old))^v).volume();rows[k]=delta;assert delta<.01,(k,delta)
# The pin-support material and roots remain unchanged near each attachment.
for x,n in [(-11,'Carriage bearing end'),(11,'Carriage body')]:
 v=cy(4.49,16.5,22.39,1,[x,0,32]);lost=((load(n)-by[n])^v).volume();assert lost<.001;(rows.update({n+' pin support removal':lost}))
# Rod end interfaces / bearing-wall sliding lengths are unchanged.
rod=by['Carriage control rod'];original=load('Carriage control rod');delta=(rod-original)+(original-rod)
assert (delta^box([-60,-10,20],[-7,30,40])).volume()<.001
assert (delta^box([7,-10,20],[60,30,40])).volume()<.001
# Test positive mechanical stops against rotation about the individual pin axes.
restraints=[]
for x,n in [(-11,'Carriage bearing end'),(11,'Carriage body')]:
 for sign in [-1,1]:
  first=None
  for deg in np.arange(.25,5.01,.25):
   M=trimesh.transformations.rotation_matrix(math.radians(sign*deg),[0,1,0],[x,14.6,32]);hit=(by[n].transform(M[:3])^rod).volume()
   if hit>.02:first=float(deg);break
  assert first is not None and first<=2.5,(n,sign,first);restraints.append(dict(part=n,sign=sign,first_contact_angle_deg=first))
(O/'Rod tie functional checks.json').write_text(json.dumps(dict(passed=True,preserved_regions_mm3=rows,anti_rotation=restraints,limits='Geometric fit and positive stop tests; no stiffness, friction, fatigue or pull-out testing.'),indent=2));print('FUNCTIONAL',rows,restraints,flush=True)
