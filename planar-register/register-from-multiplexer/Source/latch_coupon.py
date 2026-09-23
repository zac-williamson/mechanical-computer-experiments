"""Print-and-test latch cartridge, NOT an integrated or torque-qualified register.
Independent source. Only frame joints use LEGO 2780 friction pins. No printed pins.
A manually driven coupon establishes fits/capture before any actuator integration.
"""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Latch coupon';O.mkdir(exist_ok=True);P=O/'Print parts';P.mkdir(exist_ok=True)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cylinder(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=80)
 if axis==1:s=s.rotate([-90,0,0])
 if axis==0:s=s.rotate([0,90,0])
 t=list(c);t[axis]=a;return s.translate(t)
def mesh(s):
 z=s.to_mesh64();return trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False)
# 8.6mm locking pitch deliberately uses near-full original travel. It does not assume
# the supplied phase-dependent 3.749/-3.756 endpoints are exact physical stops.
keeper=box([-20,0,0],[20,4.8,12])
for x in [-4.3,4.3]:keeper-=box([x-2.35,-.1,3.2],[x+2.35,4.9,8.8])
front=box([-34,-4.4,2],[34,-.4,10])+box([-6,-4.4,0],[6,-.4,12])
for a,b in [(-34,-26),(26,34)]:front+=box([a,-4.4,-4],[b,2,16])
for a,b in [(-4,-.4),(12.4,16)]:front+=box([-26,-4.4,a],[26,2,b])
for a,b in [(-26,-24.6),(24.6,26)]:front+=box([a,-4.4,-.4],[b,2,12.4])
front-=box([-2.35,-5,3.2],[2.35,3,8.8])
for x in [-30,30]:
 front-=cylinder(2.45,-4.5,2.1,1,[x,0,6])
 front-=cylinder(3.2,1.6,2.1,1,[x,0,6])
# Positive square registration at side columns: separate from the friction pins.
# One front tongue/rear pocket pair at each side prevents relative vertical drift.
for x in [-30,30]:front+=box([x-2,2,9],[x+2,3.6,13])
rear=front.mirror([0,1,0]).translate([0,4.8,0])
# Remove rear tongues; receive front tongues with 0.4mm clearance.
for x in [-30,30]:
 rear-=box([x-2.01,1.19,8.99],[x+2.01,2.81,13.01])
 rear-=box([x-2.4,2.79,8.6],[x+2.4,4.0,13.4])
# A chamfer only in X keeps the entire bolt's underside flat on the print bed.
poly=np.array([[-1.95,-12.1],[1.95,-12.1],[1.95,8],[1.35,9.2],[-1.35,9.2],[-1.95,8]])
bolt=m.CrossSection([poly]).extrude(4.8).translate([0,0,3.6])
bolt+=box([-11.2,-20,3.6],[11.2,-12,10.8])
for x in [-6.4,6.4]:bolt-=cylinder(2.45,3.5,10.9,2,[x,-16,0])
parts={'Front guide':front,'Rear guide':rear,'Memory keeper':keeper,'Sliding bolt':bolt}
meta=[];plate=[];cursor=0
for n,s in parts.items():
 t=mesh(s);assert t.is_watertight and len(t.split())==1,n;t.export(O/(n+'.stl'))
 pt=t.copy()
 if n in ['Front guide','Memory keeper']:pt.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
 if n=='Rear guide':pt.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[1,0,0]))
 pt.apply_translation(-pt.bounds[0]);pt.export(P/(n+'.stl'));ext=pt.extents
 pp=pt.copy();pp.apply_translation([cursor,0,0]);plate.append(pp);cursor+=ext[0]+6
 bottom=pt.triangles[np.all(np.abs(pt.triangles[:,:,2])<1e-6,axis=1)]
 area=float(np.linalg.norm(np.cross(bottom[:,1]-bottom[:,0],bottom[:,2]-bottom[:,0]),axis=1).sum()/2)
 meta.append(dict(name=n,size_mm=t.extents.tolist(),print_size_mm=ext.tolist(),volume_mm3=float(t.volume),watertight=bool(t.is_watertight),connected_solids=len(t.split()),bed_contact_mm2=area))
trimesh.util.concatenate(plate).export(O/'Print layout.stl')
# Exact manifold intersection, fixed pieces and actual sampled capture/withdrawal.
def volume(a,b):return max(0.,float((a^b).volume()))
hits=[]
for q in np.linspace(-4.6,4.6,185):
 k=keeper.translate([float(q),0,0])
 for n,s in [('Front guide',front),('Rear guide',rear),('unlocked bolt',bolt.translate([0,-10,0]))]:
  v=volume(k,s)
  if v>1e-5:hits.append(dict(q=float(q),a='keeper',b=n,volume=v))
for s in np.linspace(-10,0,201):
 b=bolt.translate([0,float(s),0])
 for n,f in [('Front guide',front),('Rear guide',rear)]:
  v=volume(b,f)
  if v>1e-5:hits.append(dict(s=float(s),a='bolt',b=n,volume=v))
 for q in [-4.3,4.3]:
  v=volume(b,keeper.translate([q,0,0]))
  if v>1e-5:hits.append(dict(s=float(s),q=q,a='bolt',b='keeper',volume=v))
# Prescribed chamfer capture paths, with .03mm contact clearance for robust boolean checks.
capture=[]
for target in [-4.3,4.3]:
 for error in [-.9,-.55,0,.55,.9]:
  initial=target+error
  if abs(initial)>4.6:continue
  for s in np.linspace(-10,0,201):
   at_front_width=min(3.9,2.7+max(0,9.2+s))
   allowed=(4.7-at_front_width)/2-.03
   e=np.sign(error)*min(abs(error),allowed) if s>=-9.2 else error
   q=target+e
   v=volume(bolt.translate([0,float(s),0]),keeper.translate([float(q),0,0]))
   if v>1e-5:capture.append(dict(target=target,initial=initial,s=float(s),q=float(q),volume=v))
fixed_overlap=volume(front,rear)
# Each tooth/keeper web is checked for the ENTIRE force: no assumed equal load sharing.
F=100.;w=3.9;d=4.8;t=4.8;span=t+.8;web=8.6-(w+.8);window=d+.8
report=dict(status='FIT AND LOAD TEST COUPON ONLY — not a complete register',part_checks=meta,fixed_overlap_mm3=fixed_overlap,motion_interferences=hits,capture_path_interferences=capture,sample_counts=dict(keeper_positions=185,bolt_positions=201,capture_paths=6),clearance_per_side_mm=.4,keeper_travel_mm=[-4.6,4.6],lock_centres_mm=[-4.3,4.3],bolt_retraction_mm=10.,nominal_X_play_mm=.8,minimum_tip_clearance_unlocked_mm=.8,analysis=dict(force_N=F,assumed_bending_allowable_MPa=12,assumed_modulus_MPa=1800,bolt_bending_MPa=1.5*F*span/(d*w*w),keeper_web_point_load_bending_MPa=1.5*F*window/(t*web*web),bolt_double_shear_MPa=F/(2*w*d),keeper_contact_MPa=F/(t*d),bolt_deflection_mm=F*span**3/(4*1800*d*w**3),notes='Simply supported short-beam screening; ignores stress concentration, anisotropy, wear, joint flexibility and creep. 100N is a design study load, NOT a rated capacity or a justified bound on register force.'),unresolved=['Actuator and cam integration absent.','Frame and friction-pin joint load capacity not qualified.','Latch closure in mid-stroke deliberately obstructs; requires a compliant lost-motion drive.','No 0.2Nm system qualification.','No friction/contact-dynamics solution: capture paths are prescribed possible configurations.'])
(O/'Checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
assert not hits and not capture and fixed_overlap<1e-5,'Coupon geometry failed'
