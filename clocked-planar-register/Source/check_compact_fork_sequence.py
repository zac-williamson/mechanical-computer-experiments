"""One-sided float cannot extend a clutch's powered region toward bolt lock.

For master r=bar-lag with lag in [0,4], r>=pickup implies bar>=pickup.
For slave r=bar+lag, r<=-pickup implies bar<=-pickup. This implication covers
all float positions continuously. Cam clearance is additionally sampled with
explicit, unmeasured manufacturing allowances. No loaded release proof.
"""
from pathlib import Path
import json,hashlib
import numpy as np
from compact_cam import lifts
R=Path(__file__).resolve().parents[1]/'Compact layout';rows=[]
for pickup in [5.8,6.8,7.8]:
 for error in [-.25,0,.25]:
  worst=99.;at=None
  for s in np.linspace(-9.42,9.42,1885):
   h=lifts(s+error)
   for bank,active in [('master',s>=pickup),('slave',s<=-pickup)]:
    if active and h[bank]-3.2-.2<worst:worst=h[bank]-3.2-.2;at=[s,bank]
  rows.append(dict(pickup_mm=pickup,cam_phase_error_mm=error,vertical_error_mm=.2,min_bolt_clearance_mm=worst,at=at))
profiles=json.loads((R/'Clutch face phase study.json').read_text())['profiles'];clear=set(next(x for x in profiles if x['overlap_depth_mm']==1)['clear_phases_deg'])
flags=[all(((i+j)%720)/2 in clear for j in [-1,0,1]) for i in range(720)];best=run=0
for ok in flags*2:
 run=0 if ok else run+1;best=max(best,run)
r=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),lost_motion_mm=4,positive_withdrawal_implication=True,cam_clearance=rows,cam_bounds_pass=all(x['min_bolt_clearance_mm']>0 for x in rows),largest_sampled_blocked_phase_arc_deg=min(best,720)*.5,phase_wait_assumption='Stationary disconnected worm axle, rotating input gear; no coasting/inertia claim.',mechanically_qualified=False)
(R/'Floating fork sequence.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
