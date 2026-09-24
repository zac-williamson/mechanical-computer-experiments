"""Continuous swept-envelope separation of the D header and master gate ring.

Rotation about X preserves axial intervals and radial envelopes. Separation
in either space proves clearance for all angles and all stated rail/float
positions. This checks the named pairs, not other assembly interfaces.
"""
from pathlib import Path
import json,hashlib
import numpy as np
R=Path(__file__).resolve().parents[1]/'Compact layout'
parts={p['id']:p for p in json.loads((R/'parts.json').read_text())}
v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
def vertices(name):
 p=parts[name];return v[p['offset']//3:p['offset']//3+p['vertices']]
ring=vertices('master_gate L099');rc=np.array([10.2,16.]);rr=float(np.linalg.norm(ring[:,1:]-rc,axis=1).max())
# Include +-10 mm sequencer travel AND an independent 4 mm fork float.
# This conservative superset is wider than the physically allowed one-sided lag.
travel=14.;rows=[]
for name in ['D header input 8T','D header output 8T','D external input 4L','D input retainer']:
 a=vertices(name);c=np.array(parts[name]['centre'][1:]);radius=float(np.linalg.norm(a[:,1:]-c,axis=1).max())
 radial=float(np.linalg.norm(c-rc))-rr-radius
 axial=max(float(ring[:,0].min())-travel-float(a[:,0].max()),float(a[:,0].min())-float(ring[:,0].max())-travel)
 gap=max(axial,radial)
 rows.append(dict(part=name,axis_distance_mm=float(np.linalg.norm(c-rc)),ring_radius_mm=rr,part_radius_mm=radius,axial_gap_mm=axial,radial_gap_mm=radial,minimum_envelope_gap_mm=gap,remaining_with_0_25_mm_error_per_part=gap-.5))
r=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),certified_ring_translation_range_mm=[-travel,travel],pairs=rows,continuous_separation_pass=all(p['minimum_envelope_gap_mm']>0 for p in rows),assumed_error_bound_pass=all(p['remaining_with_0_25_mm_error_per_part']>0 for p in rows),mechanically_qualified=False)
(R/'D route clearance checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
