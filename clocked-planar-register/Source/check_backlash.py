from pathlib import Path
from itertools import product
import json
from operation import run
R=Path(__file__).resolve().parents[1];states=list(product([0,1],repeat=3));profiles=[]
for slack,contact in product([0,30,60],[.8,1.95]):
 cases=[run(a,b,q,slack=slack,contact=contact) for a in states for b in states if a!=b for q in [0,1]]
 record=dict(slack_deg=slack,pickup_mm=contact,cases=len(cases),wrong_defined_outputs=sum(c['wrong_defined_output'] for c in cases),blocked_cases=sum(c['blocked_steps']>0 for c in cases),bolt_drive_conflicts=sum(c['drive_against_inserted_bolt']>0 for c in cases),undefined_cases=sum(c['undefined_setup_hold'] for c in cases))
 profiles.append(record);print(record,flush=True)
(R/'Backlash sensitivity.json').write_text(json.dumps(dict(scope='Rigid quasi-static sensitivity to assumed backlash/pickup; no asynchronous reversal delay, inertia or physical timing qualification',profiles=profiles),indent=2))
