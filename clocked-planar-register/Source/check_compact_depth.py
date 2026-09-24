"""Measure real Y-envelope/material changes against the saved deep candidate."""
from pathlib import Path
import json, hashlib
import numpy as np
import trimesh

R=Path(__file__).resolve().parents[1]/'Compact layout'
B=Path(__file__).resolve().parents[3]/'work/compact-y-baseline'
def read(root):
 ps=json.loads((root/'parts.json').read_text())
 v=np.load(root/'geometry.npz')['vertices'].reshape(-1,3)
 return {p['id']:(p,v[p['offset']//3:p['offset']//3+p['vertices']]) for p in ps}
before,after=read(B),read(R)
def metrics(data):
 allv=np.concatenate([a for p,a in data.values()])
 volumes={};moving=0.;fixed=0.
 for name,(p,a) in data.items():
  if p['kind']!='printed':continue
  t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  assert t.is_watertight,name
  vol=abs(t.volume);volumes[name]=vol
  if p.get('motion')!='fixed':moving+=vol
  else:fixed+=vol
 return dict(envelope_mm=np.ptp(allv,axis=0).tolist(),printed_moving_volume_mm3=moving,
             printed_fixed_volume_mm3=fixed,volumes_mm3=volumes)
a,b=metrics(before),metrics(after)
unchanged=[];changed=[];key_phase_changes=[]
for name,(p,verts) in before.items():
 # Native copied actuator/clutch hardware and all external signal shafts/gears.
 core=p['kind']=='native' and ('source' in p or (p.get('axis')==0 and p.get('lego_part')!='2780'))
 if not core:continue
 if name not in after:
  changed.append(name);continue
 ap,av=after[name];av=av.copy()
 if ap.get('key_phase_deg'):
  key_phase_changes.append(dict(part=name,rotation_deg=ap['key_phase_deg']))
  av=trimesh.transform_points(av,trimesh.transformations.rotation_matrix(np.radians(-ap['key_phase_deg']),np.eye(3)[ap['axis']],ap['centre']))
 if verts.shape!=av.shape or not np.allclose(verts,av,atol=1e-5,rtol=0):changed.append(name)
 else:unchanged.append(name)
# Report changed hardware explicitly; required shaft support corrections are
# not part of the earlier depth-only revision and must not be hidden.
allowed={'WRITE selector output left','WRITE selector output right',
         'D header input 8T','D header output 8T',
         'master output left 3L','slave output left 3L',
         'master output right 7L','slave output right 8L',
         'master_gate worm drive 11L','slave_gate worm drive 10L'}
assert not set(changed)-allowed,changed
report=dict(geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),
 baseline=a,current=b,unchanged_core_hardware=unchanged,changed_core_hardware=changed,keyed_assembly_phase_changes=key_phase_changes,
 y_reduction_percent=100*(1-b['envelope_mm'][1]/a['envelope_mm'][1]),
 moving_printed_material_reduction_percent=100*(1-b['printed_moving_volume_mm3']/a['printed_moving_volume_mm3']),
 fixed_printed_material_reduction_percent=100*(1-b['printed_fixed_volume_mm3']/a['printed_fixed_volume_mm3']),
 qualification='CAD material volume, not sliced mass or stiffness/strength qualification.')
(R/'Depth reduction checks.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ['baseline','current','unchanged_core_hardware']},indent=2))
print('Envelope before/after',a['envelope_mm'],b['envelope_mm'])
