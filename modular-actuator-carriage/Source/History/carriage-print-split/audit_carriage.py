from pathlib import Path
import os,json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=Path(os.environ.get('PLANAR_OUTPUT',str(R/'split-candidate')));B=R/'carriage-print-split/before'
def load(p):
 t=trimesh.load(p);return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(rad,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,rad,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
ps={n:load(O/(n+'.stl')) for n in ['Carriage body','Carriage bearing end']};whole=sum(ps.values(),m.Manifold());old=sum((load(B/(n+'.stl')) for n in ps),m.Manifold());removed=old-whole
# Bound the rear contact region using clipped mesh edges at Y=30 over
# every source trace pose. The joint and removed far-left post must lie outside it.
lever=trimesh.load(B/'Actuator lever.stl');edges=lever.edges_unique;v=lever.vertices
trace=json.loads((R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Switching trace.json').read_text())['frames'];cache={};min_contact=float('inf')
for f in trace:
 if abs(f['q'])>3.75:continue
 angle=float(f['b'])
 if angle not in cache:
  b=np.deg2rad(-angle);c=np.cos(b);ss=np.sin(b)
  x=13.192323604+c*(v[:,0]-13.192323604)-ss*(v[:,1]-26.328448698)
  y=26.328448698+ss*(v[:,0]-13.192323604)+c*(v[:,1]-26.328448698)
  values=x[y>=30].tolist();e=edges[(y[edges[:,0]]>=30)!=(y[edges[:,1]]>=30)]
  if len(e):values.extend((x[e[:,0]]+(30-y[e[:,0]])*(x[e[:,1]]-x[e[:,0]])/(y[e[:,1]]-y[e[:,0]])).tolist())
  cache[angle]=min(values) if values else float('inf')
 min_contact=min(min_contact,cache[angle]-f['q'])
active_min=min_contact-.2
checks=[dict(check='Joint outside rear lever contact region',minimum_contact_x=min_contact,checked_region_starts_x=active_min,joint_ends_x=-1.6,passed=active_min>-1.6)]
for label,region in [('Original clutch contact material',cy(7.4,-8,7.8,0,[0,10.2,0])),('Lever rear working profile in active X range',box([active_min,30,11.6],[30,37.49,20]))]:
 v=(removed^region).volume();checks.append(dict(check=label,removed_mm3=v,passed=v<.02))
for n,a,b in [('Carriage bearing end',-16.2,-9.71),('Carriage body',9.71,16.2)]:
 v=(ps[n]^cy(2.83,a,b,0,[0,10.2,16])).volume();checks.append(dict(check=n+' clear vertical axle bore',intrusion_mm3=v,passed=v<.02))
strength=[]
def probe(n,label,s):
 d=s-ps[n];v=d.volume();strength.append(dict(part=n,region=label,missing_mm3=v,missing_bounds=d.bounding_box() if v>=.02 else None,passed=v<.02))
for n,a,b,x in [('Carriage bearing end',-16.2,-9.71,-11),('Carriage body',9.71,16.2,11)]:
 probe(n,'Axle bearing: 3.05 mm radial material',cy(5.95,a,b,0,[0,10.2,16])-cy(2.9,a-.01,b+.01,0,[0,10.2,16]))
 probe(n,'Rod pin: 2 mm surround through supported grip',cy(4.49,15.11,22.39,1,[x,0,32])-cy(2.51,15.1,22.4,1,[x,0,32]))
 probe(n,'Rod collar: 2 mm surround',cy(5.24,14.41,15.09,1,[x,0,32])-cy(3.26,14.4,15.1,1,[x,0,32]))
 aa,bb=(-2,-1.81) if x<0 else (-1.59,-1.4)
 for y,z,label in [(24.7,.5,'Lower'),(35.4,32,'Upper')]:probe(n,label+' joining collar: 2 mm surround',cy(5.29,aa,bb,0,[0,y,z])-cy(3.31,aa-.01,bb+.01,0,[0,y,z]))
 aa,bb=(-16.2,-1.81) if x<0 else (-1.59,16.2)
 probe(n,'Rear spine: 3.3 mm Y core',box([aa,40.4,8],[bb,43.7,25]))
 probe(n,'Lower rear rail: 3 mm Z core',box([aa,30.3,2.1],[bb,43.7,5.1]))
 probe(n,'Upper rear rail: 3 mm Z core',box([aa,32,36],[bb,34,39]))
 probe(n,'Front upper rail: 3 mm Y by 2.7 mm Z core',box([aa,1.6,24.9],[bb,4.6,27.6]))
(O/'Carriage interface checks.json').write_text(json.dumps(checks,indent=2));(O/'Carriage strength probes.json').write_text(json.dumps(strength,indent=2))
print('Interface checks',json.dumps(checks,indent=2),flush=True);print('Strength failures',json.dumps([a for a in strength if not a['passed']],indent=2),flush=True)
# A small sample of the actual bearing transition, in the actual bed orientation.
coupon=ps['Carriage bearing end']^box([-16.25,1.55,9.15],[-1.8,16.25,24.65])
coupon=coupon.rotate([0,90,0]);bb=coupon.bounding_box();coupon=coupon.translate([-bb[0],-bb[1],-bb[2]])
import ast,sys
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R/'adapt.py').read_text());defs=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ['solid','mesh','add']]
env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[])
exec(compile(ast.Module(body=defs,type_ignores=[]),'coupon-export','exec'),env)
env['add']('Bearing print test',coupon);t=env['parts'][0]['mesh'];t.export(O/'Bearing print test.stl')
assert trimesh.load(O/'Bearing print test.stl').is_watertight
(O/'Bearing print test.json').write_text(json.dumps(dict(watertight=t.is_watertight,bounds=t.bounds.tolist(),note='Check the unsupported thrust-shoulder ledge and vertical bore on this small sample before printing a carriage. Not a printability guarantee.'),indent=2))
assert all(a['passed'] for a in checks+strength)
