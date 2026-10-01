from pathlib import Path
import os,runpy,json,shutil,hashlib,csv
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'reliability-candidate';os.environ['PLANAR_OUTPUT']=str(O)
assert json.loads((O/'Bolt print fix checks.json').read_text())['passed']
for name in ['carriage-print-split/audit_carriage.py','reliability/mechanism.py']:
 runpy.run_path(str(R/name),run_name='__main__')
def load(n):return trimesh.load(O/(n+'.stl'))
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
clutch=runpy.run_path(str(R/'native_envelopes.py'))['clutch_envelope']
D=json.loads((O/'Model.json').read_text());import gzip,base64
v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);p=next(p for p in D['parts'] if p['name']=='L099');a=v[p['offset']//3:p['offset']//3+p['vertices']];native=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
fork=solid(load('Carriage body'))+solid(load('Carriage bearing end'));margins=[]
for r in [0,.1,.15,.2]:
 s=clutch(native,r);margins.append(dict(radial_allowance_mm=r,overlap_mm3=(s^fork).volume(),envelope_volume_mm3=s.volume()))
assert margins[0]['overlap_mm3']<.02
# Verify the actual lock tip clears both pockets over their nominal lateral play.
bolt=solid(load('Locking bolt'));lock=[]
for endpoint in [-3.75,3.75]:
 for error in [-.19,0,.19]:
  q=endpoint+error;hit=(bolt^fork.translate([q,0,0])).volume();lock.append(dict(carriage_X=q,locked_overlap_mm3=hit));assert hit<.02
# Clear tip at full release, for every carriage position across the stroke.
for q in np.linspace(-3.75,3.75,31):assert (bolt.translate([0,0,3.8])^fork.translate([float(q),0,0])).volume()<.02
(O/'Functional margins.json').write_text(json.dumps(dict(clutch_radial_clearance=margins,lock_play_checks=lock,release_positions=31,nominal_lock_tip_engagement_mm=3,nominal_released_tip_gap_mm=.8,limits='Geometric fits only. Gear tooth phases, torque, elastic preload and printed dimensions remain physical test items.'),indent=2))
# Print small sections of the actual parts, in the same orientations as the full parts.
coupons=[('Rod guide fit sample',solid(load('Right bearing wall'))^box([20.4,2.2,25.5],[24.4,19.2,38.5]),[-1,0,0]),('Rod fit sample',solid(load('Carriage control rod'))^box([16,6.1,28.1],[30,14.3,35.9]),[0,1,0]),('Lock pocket fit sample',solid(load('Carriage bearing end'))^box([-13,20.5,33],[-4.5,31.5,39.5]),[1,0,0]),('Fixed axle bearing fit sample',solid(load('Right bearing wall'))^box([20.4,3,-6.5],[24.4,17.2,6.5]),[-1,0,0])]
placed=[];rows=[];cursor=0
# Existing bearing coupon uses the actual annular thrust-shoulder ledge.
t=load('Bearing print test');t.apply_translation(-t.bounds[0]);placed.append(t);cursor=t.extents[0]+7;rows.append(dict(part='Bearing print test',bounds=t.bounds.tolist()))
for name,s,normal in coupons:
 a=s.to_mesh64();t=trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=True);assert t.is_watertight and len(t.split())==1,name
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(O/(name+'.stl'));w=t.extents[0];t.apply_translation([cursor,0,0]);cursor+=w+7;placed.append(t);rows.append(dict(part=name,bounds=t.bounds.tolist()))
layout=trimesh.util.concatenate(placed);layout.export(O/'Fit test layout.stl');(O/'Fit test layout.json').write_text(json.dumps(dict(bounds=layout.bounds.tolist(),parts=rows,notes='Five small actual-geometry samples. Print the bearing sample first to judge its annular underside. Do not treat successful slicing as measured fit.'),indent=2))
# Current surface audit and corrected moving-bolt connectivity report.
s=(R/'design-review/print_audit.py').read_text().replace("R/'design-review/Print geometry audit.json'","O/'Current print audit.json'")
exec(compile(s,'final print audit','exec'),{'__name__':'__main__','__file__':str(R/'design-review/print_audit.py')})
shutil.copy2(O/'Current layer audit.json',O/'Before layer audit.json');rows=json.loads((O/'Current layer audit.json').read_text())
for row in rows:
 if row['part']=='Locking bolt':
  # Re-screen only the changed part. Other part meshes are unchanged.
  t=load('Locking bolt');t.apply_transform(trimesh.geometry.align_vectors([0,-1,0],[0,0,-1]));t.apply_translation(-t.bounds[0]);ss=solid(t);events=[]
  for h in np.arange(.3,t.extents[2],.2):
   prev=ss.slice(h-.2);now=ss.slice(h);extra=now-prev.offset(.21);islands=[c.area() for c in now.decompose() if c.area()>.1 and (c^prev.offset(.21)).area()<.01]
   if extra.area()>.5 or islands:events.append(dict(h=float(h),overhang_mm2=extra.area(),islands=islands))
  row['events']=events
(O/'Current layer audit.json').write_text(json.dumps(rows,indent=2));assert all(not e['islands'] for row in rows for e in row['events'])
runpy.run_path(str(R/'render_review.py'),run_name='__main__')
(O/'Checked mesh hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.stl')},indent=2))
print('Functional margins, five-sample fit layout and final audit complete.',flush=True)
