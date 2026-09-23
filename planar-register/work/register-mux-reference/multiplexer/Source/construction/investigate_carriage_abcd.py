"""Independent additions test the annotated roof, fork-root and guide reliefs."""
from geometry import *
from clean_print_mesh import clean
from trimesh.collision import CollisionManager
O=OUT/'Flat-actuator';D=O/'Two-piece exploration'
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
t=trimesh.load(D/'Left merged candidate.stl');base=solid(t)
# Extend the left end of the existing stop lobe down to the print plane.
zmin=float(t.vertices[(t.vertices[:,2]>35)&(t.vertices[:,0]>-2.21)&(t.vertices[:,0]<10.4)&(t.vertices[:,1]<20.26),2].min())
A=box([-15.6,11.85,zmin],[-2.2,20.25,43.79955051738877])-base
# Extend the suspended fork's -X face to the same print plane.
mask=(t.face_normals[:,0]<-.99)&(abs(t.triangles_center[:,0]+1.6)<.01)
profile=unary_union([Polygon(tri[:,[1,2]]) for tri in t.triangles[mask]])
C=extr(profile,0,14.01).rotate([0,90,0]).rotate([90,0,0]).translate([-15.6,0,0])-base
# Fill the longitudinal opening between the two carriage crossbars.
Dfill=box([-15.6,20.8,5.8],[7.8,27.8,11.0])-base
trials={'AB roof extension':A,'C fork extension':C,'C curved relief fill':box([-15.6,11.4,5.5],[-7.3,20.2,9.15])-base,'D guide opening fill':Dfill}
meta=json.loads((O/'printed-parts.json').read_text());fixed={p['id']:solid(trimesh.load(O/(p['id']+'.stl'))) for p in meta if p['motion']=='fixed'}
hm=json.loads((O/'hardware.json').read_text());vv=np.load(O/'hardware.npz')['vertices'];ch=CollisionManager()
for p in hm:ch.add_object(p['id'],trimesh.Trimesh(vv[p['offset']//3:p['offset']//3+p['vertices']],np.arange(p['vertices']).reshape(-1,3),process=False))
params=json.loads((O/'parameters.json').read_text());pv=np.array(params['pivot']);lever=solid(trimesh.load(O/'Short lever.stl'));rows=json.loads((OUT/'Local-support'/'Switching trace.json').read_text())['frames'];results={}
for label,added in trials.items():
 fixedhits={};leverhits=[];native={};cm=CollisionManager();cm.add_object(label,mesh(added))
 for q in np.linspace(-4.6,4.575,185):
  for name,f in fixed.items():
   volume=(added.translate([q,0,0])^f).volume()
   if volume>.001:fixedhits[name]=max(fixedhits.get(name,0),volume)
 seen=set()
 for r in rows:
  q,b=r['q'],r['b'];key=(round(q,2),round(b,1))
  if key in seen:continue
  seen.add(key);volume=(added.translate([q,0,0])^lever.translate(-pv).rotate([0,b,0]).translate(pv)).volume()
  if volume>.001:leverhits.append(dict(q=q,beta=b,volume=volume))
 for r in rows[::max(1,len(rows)//180)]:
  q,b,w,g=r['q'],r['b'],r['w'],r['g'];T=np.eye(4);T[0,3]=q;cm.set_transform(label,T)
  for p in hm:
   mo=p['motion'];T=np.eye(4)
   if mo=='gear':T=trimesh.transformations.rotation_matrix(np.radians(g),[0,1,0],params['reaction_center'])
   elif mo in ['input','worm']:
    T=trimesh.transformations.rotation_matrix(np.radians(w),[1,0,0],params['worm_center'])
    if mo=='worm':T[0,3]=q
   elif mo=='carriage':T[0,3]=q
   elif mo=='clutch-ring':T[0,3]=np.sign(q)*max(abs(q)-.4,0)
   ch.set_transform(p['id'],T)
  _,pairs=cm.in_collision_other(ch,return_names=True)
  for _,name in pairs:native[name]=native.get(name,0)+1
 results[label]=dict(added_mm3=added.volume(),fixed_overlap_max_mm3=fixedhits,lever_collisions=leverhits,native_contacts=native)
 mesh(added).export(D/(label+' - addition.stl'))
 print(label,results[label],flush=True)
(D/'ABCD investigation.json').write_text(json.dumps(results,indent=2))
# Save the roof simplification separately until its checks have been assessed.
s=base+A;out=clean(mesh(s));out.export(D/'Simplified roof candidate.stl',file_type='stl_ascii');out.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]));out.apply_translation(-out.bounds[0]);out.export(D/'Simplified roof candidate - print.stl',file_type='stl_ascii')
print('roof lower Z',zmin)
