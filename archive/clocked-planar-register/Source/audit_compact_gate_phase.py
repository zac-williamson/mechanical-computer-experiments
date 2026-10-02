"""Gate dog-phase screening of the actual recorded angular poses.

Uses conservative projected native tip profiles. A flagged phase is a
potential interference requiring 3-D confirmation, not a strength result.
"""
from pathlib import Path
import json,hashlib
from compact_dog_contact import face_clear
R=Path(__file__).resolve().parents[1]/'Compact layout';r=json.loads((R/'Compact contact-resolved operation.json').read_text());ph=json.loads((R/'External gear phase checks.json').read_text())['shaft_phases_deg'];hits=[];tested=0
for ci,c in enumerate(r['cases']):
 for fi,f in enumerate(c['frames']):
  for bank,stage,sign in [('master_gate','master',1),('slave_gate','slave',-1)]:
   position=f['rail']+f.get(bank+'_lag',0.)
   if sign*position<7.8:continue
   tested+=1;gear=(f['angles']['X'] if sign==1 else -f['angles']['M'])+ph.get(bank+' L102',0.)
   relative=f[stage]['w']-gear
   if not face_clear(relative):hits.append(dict(case=ci,frame=fi,gate=bank,position_mm=position,relative_angle_deg=relative%360))
out=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),tested_engaged_poses=tested,flagged_poses=len(hits),examples=hits[:40],projected_phase_screen_pass=not hits,mechanically_qualified=False)
(R/'Recorded gate phase screening.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
