"""Conservative quasi-static clutch/cam sequencing sensitivity, not measured timing."""
from pathlib import Path
from itertools import product
import json,hashlib,numpy as np
from compact_operation import run
from compact_cam import lifts
R=Path(__file__).resolve().parents[1]/'Compact layout'
# Native axial envelopes give 6.8 mm earliest possible nominal dog contact.
# Sweep an explicit +/-1 mm error allowance, rather than assuming full insertion.
sequence=[]
for gate_contact in [5.8,6.8,7.8]:
 for cam_shift in [-.25,0,.25]:
  minimum=1e9;conflicts=0
  for s in np.linspace(-9.4,9.4,1881):
   heights=lifts(s+cam_shift)
   for bank,active in [('master',s>=gate_contact),('slave',s<=-gate_contact)]:
    if active:
     clearance=heights[bank]-3.2-.2
     minimum=min(minimum,float(clearance));conflicts+=int(clearance<0)
  sequence.append(dict(first_possible_pickup_mm=gate_contact,cam_phase_error_mm=cam_shift,vertical_error_mm=.2,min_bolt_tip_clearance_mm=minimum,conflicts=conflicts))
scenarios=[]
for warmup,contact in [(w,6.8) for w in [0,17,43,89,137,181,271,359]]+[(89,5.8),(89,7.8)]:
 rows=[]
 for start in product([0,1],repeat=3):
  for end in product([0,1],repeat=3):
   if start==end:continue
   for q in [0,1]:rows.append(run(start,end,q,warmup=warmup,contact=contact))
 scenarios.append(dict(warmup_steps=warmup,first_possible_pickup_mm=contact,cases=112,wrong_defined_outputs=sum(x['wrong_defined_output'] for x in rows),blocked_cases=sum(x['blocked_steps']>0 for x in rows),bolt_drive_conflicts=sum(x['drive_against_inserted_bolt']>0 for x in rows)))
 print(scenarios[-1],flush=True)
report=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),sequencing=sequence,transition_scenarios=scenarios,assumptions=['Axial error allowance is a proposed tolerance budget, not measured hardware','Backlash comes from the native dog geometry; running phase varies before the input transition','No inertia, loaded dog friction, band force or rapid repeated clock edges'],mechanically_qualified=False)
(R/'Compact timing sensitivity.json').write_text(json.dumps(report,indent=2))
