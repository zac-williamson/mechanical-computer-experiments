"""Publish only the current revision and its current checks; preserve the old folder."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,gzip,base64,re,shutil,zipfile
R=Path(__file__).resolve().parents[1];O=R.parent/'work/integrated-cam-development';dest=R/'Planar register'
parts=json.loads((O/'printed-parts.json').read_text());hw=json.loads((O/'hardware.json').read_text());timing=json.loads((O/'Transition timing audit.json').read_text());rot=json.loads((O/'Rotating clearance checks.json').read_text())
assert json.loads((O/'Printed travel intersections.json').read_text())=={}
assert json.loads((O/'Transition printed intersections.json').read_text())=={}
assert not rot['penetrations']
hits=json.loads((O/'Development hardware intersections.json').read_text());unexpected={n:r for n,r in hits.items() if ' pin ' not in n.split(' / ')[1] or r['depth']>.17};assert not unexpected,unexpected
hashes={}
for p in parts:
 t=trimesh.load(O/(p['id']+'.stl'));assert t.is_watertight and len(t.split())==1;hashes[p['id']]=hashlib.sha256((O/(p['id']+'.stl')).read_bytes()).hexdigest()
html=(O/'Viewer.html').read_text();D=json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',html,re.S).group(1));v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);lo=np.array([np.inf]*3);hi=-lo
for p in D['parts']:
 a=v[p['offset']//3:p['offset']//3+p['vertices']];l=a.min(0).astype(float);h=a.max(0).astype(float)
 if p['motion'] in ['carriage','worm']:l[0]-=3.75;h[0]+=3.75
 if p['motion']=='clutch-ring':l[0]-=4.15;h[0]+=4.15
 if p['motion'] in ['bolt','lock-band']:h[2]+=5.4
 lo=np.minimum(lo,l);hi=np.maximum(hi,h)
verification=dict(status='REJECTED AS FUNCTIONALLY VALIDATED; assembly print recommendation withdrawn',printed_parts=len(parts),hardware_parts=len(hw),printed_grid_poses=1681,grid_step_mm=.1875,printed_transition_poses=len({(round(f['qm'],5),round(f['qe'],5),round(f['s'],5)) for c in timing['cases'] for f in c['frames']}),timing_examples=len(timing['cases']),input_transitions=len(timing['transitions']),timing_frames=sum(len(c['frames']) for c in timing['cases']),unintended_printed_intersections=[],printed_intersection_volume_threshold_mm3=.01,hardware_sampling='Nine carriage positions per actuator; native vertices tested against closed printed solids. Rocker/native and native/native physical contacts not established by this test.',native_friction_pin_contact_pairs=len(hits),max_sampled_friction_pin_interior_mm=max(r['depth'] for r in hits.values()),unexpected_sampled_native_printed_contacts=unexpected,drive_rotation_poses=rot['poses'],drive_rotation_parts=rot['hardware_parts'],drive_rotation_penetrations=rot['penetrations'],envelope_of_translating_parts_mm=(hi-lo).tolist(),envelope_bounds_mm=[lo.tolist(),hi.tolist()],envelope_limit='Includes nominal drive mesh extents and translational stroke; not a tolerance or full rotational swept envelope.',printed_sha256=hashes,physical_0_1_Nm_qualified=False,continuous_collision_proof=False,frame_FEA_performed=False,simultaneous_closing_data_race_resolved=False)
(O/'Revision verification.json').write_text(json.dumps(verification,indent=2))
archive=R/'Superseded cam revision before integrated carriage'
if not archive.exists():shutil.move(dest,archive)
dest.mkdir(exist_ok=True)
files=[p['id']+'.stl' for p in parts]+['printed-parts.json','hardware.json','hardware.npz','Integrated cam parameters.json','Integrated loading screen.json','Printed travel intersections.json','Transition printed intersections.json','Development hardware intersections.json','Rotating clearance checks.json','Actuator addition checks.json','Transition timing audit.json','Transition timing audit.md','Clutch phase contact scan.json','Cross-core hardware check.json','Drive coupling check.json','Print orientations.json','Fit coupon inventory.json','Revision verification.json','Functional validation gaps.json','Viewer.html','README.md']
for name in ['Coupled operation.json','Coupled viewer parts.json','Coupled model checks.json','Coupled assembly contacts.json','Keyed hardware alignment.json','Connected gear phases.json','Clutch axle stop correction.json']:
 if (O/name).exists():files.append(name)
if (O/'Roller passage relief.json').exists():files.append('Roller passage relief.json')
for name in files:shutil.copy2(O/name,dest/name)
for name in ['Prototype print parts','Quick lock fit test']:shutil.copytree(O/name,dest/name,dirs_exist_ok=True)
with zipfile.ZipFile(dest/'Integrated cam mechanical test.zip','w',zipfile.ZIP_DEFLATED) as z:
 for name in files:
  if name.endswith('.stl') or name=='hardware.npz':continue
  z.write(dest/name,name)
 for folder in ['Prototype print parts','Quick lock fit test']:
  for p in (dest/folder).iterdir():z.write(p,str(p.relative_to(dest)))
print(json.dumps(verification,indent=2))
