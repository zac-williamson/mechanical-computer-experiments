"""Angle-driven protocol sweep using the planar actuator contact model.
This is rigid/quasi-static and does not establish torque or real timing bounds.
"""
from pathlib import Path
from itertools import product
import json,math,time
import numpy as np
from actuator import Actuator,tables
from sequence import pose,selected_data
R=Path(__file__).resolve().parents[1]
T=tables()
class Clutch:
 def __init__(self,slack):self.slack=slack;self.remaining=0.;self.active=False;self.sign=0
 def step(self,delta,active):
  if not active or delta==0:self.active=False;return 0.
  sign=1 if delta>0 else -1
  if not self.active or sign!=self.sign:self.remaining=self.slack
  self.active=True;self.sign=sign
  take=min(abs(delta),self.remaining);self.remaining-=take
  return sign*(abs(delta)-take)
def ring(q,r):return min(q+.4,max(q-.4,r))
def pocket(q):return min(abs(-5.05-(x+q)) for x in [-9,-1.1])<=.95

def run(start,end,q0,slack=60,contact=.8,steps=2400,keep=False):
 d,w,k=start;dd,ww,kk=end
 m=Actuator(1-2*(selected_data(d,w,q0) if k==0 else q0),T)
 o=Actuator(1-2*q0,T);we=Actuator(1-2*w,T);ck=Actuator(2*k-1,T)
 rm=m.q-math.copysign(.4,m.q);ro=o.q-math.copysign(.4,o.q);rw=we.q-math.copysign(.4,we.q)
 gates=[Clutch(slack) for _ in range(5)]
 angle=dict(D=0.,WRITE=0.,CLK=0.,POWER=0.,X=0.,M=0.,Q=0.)
 stalls=[];bad_drive=[];frames=[];first_q_change=None
 expected=None if k==0 and kk==1 and (d,w)!=(dd,ww) else selected_data(d,w,q0) if k==0 and kk==1 else q0
 for i in range(steps+1):
  control=pose(3*ck.q)
  ml=max(control['master_lift'],0 if pocket(m.q) else 3.2)
  ol=max(control['slave_lift'],0 if pocket(o.q) else 3.2)
  qbit=1 if ro < -contact else 0 if ro>contact else None
  if keep and i%6==0:
   frames.append(dict(turns=i/180,master=m.state(),slave=o.state(),write=we.state(),clock=ck.state(),
    rm=rm,ro=ro,rw=rw,rail=3*ck.q,master_lift=ml,slave_lift=ol,
    master_ring=control['master_ring'],slave_ring=control['slave_ring'],Q=qbit,angles=dict(angle)))
  if i==steps:break
  # Every tick supplies 2 degrees to D, WRITE, CLK and POWER. Time is stated
  # in input-shaft revolutions, deliberately not inferred from an unspecified motor.
  angle['D']+=2*(2*dd-1);angle['WRITE']+=2*(2*ww-1);angle['CLK']+=2*(2*kk-1);angle['POWER']+=2
  we.step(-2*(2*ww-1));ck.step(2*(2*kk-1));rw=ring(we.q,rw)
  control=pose(3*ck.q)
  # The actual planar POWER gear paths give +1/2 and -1 respectively.
  md=1 if rm < -contact else -2 if rm>contact else 0
  qd=1 if ro < -contact else -2 if ro>contact else 0
  md=gates[2].step(md,md!=0);qd=gates[3].step(qd,qd!=0)
  angle['M']+=md;angle['Q']+=qd
  xd=2*(2*dd-1) if rw < -contact else qd if rw>contact else 0
  xd=gates[4].step(xd,xd!=0)
  angle['X']+=xd
  for name,a,delta,idx in [('master',m,-xd,0),('slave',o,-md,1)]:
   lift=max(control[name+'_lift'],0 if pocket(a.q) else 3.2)
   active=control[name+'_ring']>contact
   du=gates[idx].step(delta,active)
   if active and lift<3.2-1e-8:bad_drive.append((i,name))
   if du:
    a.step(du,locked=lift<3.2-1e-8)
    if a.stalled:stalls.append((i,name,a.q))
  rm=ring(m.q,rm);ro=ring(o.q,ro)
  now=1 if ro < -contact else 0 if ro>contact else None
  if now!=q0 and first_q_change is None:first_q_change=i/180
 qfinal=1 if ro < -contact else 0 if ro>contact else None
 result=dict(start=start,end=end,initial_Q=q0,expected_Q=expected,final_Q=qfinal,
   master_q=m.q,slave_q=o.q,master_in_pocket=pocket(m.q),slave_in_pocket=pocket(o.q),
   undefined_setup_hold=expected is None,wrong_defined_output=expected is not None and qfinal!=expected,
   blocked_steps=len(stalls),drive_against_inserted_bolt=len(bad_drive),first_Q_change_input_turns=first_q_change,
   slack_deg=slack,first_contact_mm=contact)
 if keep:result['frames']=frames
 return result

if __name__=='__main__':
 cases=[];states=list(product([0,1],repeat=3));t=time.monotonic()
 for start in states:
  for end in states:
   if start==end:continue
   for q in [0,1]:
    cases.append(run(start,end,q,keep=(start==(1,1,0) and end==(1,1,1) and q==0)))
  print('Completed start',start,'elapsed',round(time.monotonic()-t,1),flush=True)
 report=dict(scope='Rigid angle-driven simulation using planar contact tables; assumed slack and dog contact',
   cases=cases,case_count=len(cases),wrong_defined_outputs=sum(c['wrong_defined_output'] for c in cases),
   blocked_cases=sum(c['blocked_steps']>0 for c in cases),bolt_drive_conflicts=sum(c['drive_against_inserted_bolt']>0 for c in cases),
   qualifications=['No inertia, friction, stiffness or dynamic tooth-impact model',
    'Fork motion is prescribed from the designed cam centreline; physical linkage containment must be checked separately',
    '60 degree slack and 0.8 mm pickup are assumed, not measured',
    'Setup/hold violation results are observations, not guarantees',
    'Gear phase, volume intersections and shaft restraint require separate CAD checks'])
 (R/'Angle-driven operation.json').write_text(json.dumps(report,separators=(',',':'),default=lambda x:x.item()))
 print({k:v for k,v in report.items() if k not in ['cases','qualifications']})
