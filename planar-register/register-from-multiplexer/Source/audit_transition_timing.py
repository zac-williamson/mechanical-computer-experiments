"""Conditional contact-event witnesses, not a force-driven qualification.
Time is normalized: WRITE carriage free-travel speed = 1 mm/time-unit.
Zero angular take-up delay is a possible flank phase, not a claimed measurement.
"""
from pathlib import Path
import json, itertools
from direct_cam_math import cam_lift
R=Path(__file__).resolve().parents[1]; O=R/'Planar register'
END=3.75; PLAY=.4; OUTER_GAP=.8
scan=json.loads((O/'Clutch phase contact scan.json').read_text())
MESH_CONTACT=scan['first_sample_with_contact_mm']
def play(q,r): return min(q+PLAY,max(q-PLAY,r))
def approach(x,y,d): return x+max(-d,min(d,y-x))
def output(r,c): return '0' if r>c+1e-6 else '1' if r< -c-1e-6 else 'undriven / phase uncertain'
def run(d0,w0,d1,w1,bit,ratio,lag,contact):
 q=END*(1-2*bit); e=END*(2*w0-1); r=q-PLAY*(1 if q>0 else -1); er=e-PLAY*(1 if e>0 else -1)
 frames=[]; disconnect=None; release=None; bind=False; oldbit=bit; initial_old_connected=True
 for i in range(2251):
  t=i*.01
  if i:
   e=approach(e,END*(2*w1-1),.01);er=play(e,er)
   driven=er>contact
   if w0 and not w1 and not driven and disconnect is None:disconnect=dict(t=t,qm=q,qe=e)
   if driven:
    d=d0 if t<lag else d1
    proposed=approach(q,END*(1-2*d),ratio*.01)
    # Do not suppress a commanded collision and then call it safe.
    if cam_lift(e)<5.4 and proposed!=q:bind=True
    q=proposed;r=play(q,r)
  driven=er>contact
  if w0 and d0!=d1 and release is None and output(r,contact)!=str(oldbit):release=dict(t=t,qm=q,qe=e)
  pocket=abs(q)>=3.3
  up=max(cam_lift(e),0 if pocket else 5.4)
  locked=pocket and up<.01
  Q=output(r,contact)
  if i%5==0 or i==2250:
   frames.append(dict(q=q,qm=q,qe=e,rm=r,re=er,s=up,b=0,w=0,g=0,angle=0,segment=f'D {d0}→{d1} · WRITE {w0}→{w1} · D clutch '+('contact possible' if driven else 'separated')+f' · Q {Q} · '+('LOCKED' if locked else 'NOT LOCKED')))
 return dict(initial=[d0,w0,bit],target=[d1,w1],ratio=ratio,reversal_delay_units=lag,contact_shift_mm=contact,disconnect=disconnect,old_output_release=release,release_before_disconnect=bool(release and disconnect and release['t']<disconnect['t']-1e-8),commanded_locked_drive=bind,final_qm=q,final_qe=e,final_Q=Q,final_locked=locked,frames=frames)
profiles=[('equal carriage speed / mesh contact',1,0,MESH_CONTACT),('equal speed / earliest envelope',1,0,OUTER_GAP),('memory 1.5× WRITE / mesh contact',1.5,0,MESH_CONTACT),('memory 0.25× WRITE / mesh contact',.25,0,MESH_CONTACT),('memory 4× WRITE / mesh contact',4,0,MESH_CONTACT),('equal speed / delayed reversal',1,4,MESH_CONTACT)]
cases=[]; summary=[]
for d0,w0,d1,w1 in itertools.product([0,1],repeat=4):
 if (d0,w0)==(d1,w1):continue
 rows=[]
 for bit in ([d0] if w0 else [0,1]):
  for name,ratio,lag,c in profiles:
   r=run(d0,w0,d1,w1,bit,ratio,lag,c);r['profile']=name;cases.append(r);rows.append(r)
 closing_change=w0==1 and w1==0 and d0!=d1
 summary.append(dict(transition=[d0,w0,d1,w1],hold_initial_bits=[d0] if w0 else [0,1],witness_count=len(rows),invalid_hold_witnesses=sum(not r['final_locked'] for r in rows) if not w1 else 0,old_output_released_before_disconnect=any(r['release_before_disconnect'] for r in rows),assessment='FAIL: race can leave unlocked/undriven memory or capture new bit' if closing_change else 'Conditional only: nominal event witnesses settle; load, angular phase and reaction-time bounds remain unqualified'))
report=dict(status='NOT READY TO PRINT: closing-data race demonstrated; absence of axle binding is not established',units='mm; time normalized to WRITE carriage speed 1 mm/unit, not seconds',axial_fork_half_play_mm=PLAY,earliest_contact_envelope_mm=OUTER_GAP,native_mesh_first_sample_contact_mm=MESH_CONTACT,nominal_bolt_first_intrusion_qe_mm=-.75,models=['Hysteretic clutch-ring play: r = clamp(previous r, q-0.4, q+0.4).','Potential torque transfer starts at axial dog contact, without requiring full seating. Angular take-up may be zero.','D remains coupled during WRITE retreat until ring loses contact. Both actuators can move concurrently.','No inertia after separation in witnesses; this optimistic assumption already produces failures.','HOLD begins with either stored bit independent of D.'],limits=['Profile ratios and delays are sensitivity examples, not measured operating bounds.','No finite maximum response time is established: tooth blocking, guide stiction, load and insufficient torque can stall.','Native open triangle meshes and 1-degree phase sampling do not establish physical angular backlash or tooth tolerance.','No force, torsion, tooth-impact, chamfer seating or loaded clutch-release solver; no axle is certified against binding.','Output contact is potential coupling; relative angular backlash and external Q load can delay direction transfer.'],transitions=summary,cases=cases)
(O/'Transition timing audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(status=report['status'],transitions=summary),indent=2))
