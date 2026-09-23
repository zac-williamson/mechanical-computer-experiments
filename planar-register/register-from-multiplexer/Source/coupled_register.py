"""Angle-driven, quasi-static register model. All visible shaft angles are recorded.
Clutch pickup/backlash are explicit sensitivity parameters, not measured bounds.
"""
from pathlib import Path
import json,itertools,math,numpy as np
from coupled_actuator import Actuator,tables,LEAD
from integrated_cam_math import cam_lift
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development'
def play(q,r):return min(q+.4,max(q-.4,r))
def pocket_clear(q):return min(abs(-5.05-(x+q)) for x in [-9,-1.1])<=.95
class Clutch:
 def __init__(self,takeup):self.active=False;self.sign=0;self.remaining=0.;self.takeup=takeup
 def step(self,delta,enabled):
  sg=1 if delta>0 else -1
  if not enabled:self.active=False;return 0.
  if not self.active or sg!=self.sign:self.remaining=self.takeup;self.active=True;self.sign=sg
  used=min(abs(delta),self.remaining);self.remaining-=used;return sg*(abs(delta)-used)
def run(d0,w0,d1,w1,bit,backlash,contact):
 t=tables();m=Actuator(1-2*bit,t);e=Actuator(2*w0-1,t);r=m.q-math.copysign(.4,m.q);er=e.q-math.copysign(.4,e.q);gate=Clutch(backlash);out=Clutch(backlash);frames=[];power=0.;data=0.;Qangle=0.;roller=0.;oldlift=cam_lift(e.q);old_e=e.q;stall=[];max_error=0
 # 60 rpm is an illustrative shaft speed. Integration step is 2 shaft degrees.
 for i in range(1801):
  if i:
   before=(e.q,e.w,e.g);e.step(2*(2*w1-1));max_error=max(max_error,abs((e.q-before[0])-LEAD/360*((e.w-before[1])-8*(e.g-before[2]))));er=play(e.q,er)
   data+=2*(2*d1-1);power+=2
   du=gate.step(-2*(2*d1-1),er>contact)
   cam=cam_lift(e.q);lift=max(cam,0 if pocket_clear(m.q) else 3.2)
   before=(m.q,m.w,m.g)
   if du:m.step(du,locked=lift<3.2-1e-6)
   max_error=max(max_error,abs((m.q-before[0])-LEAD/360*((m.w-before[1])-8*(m.g-before[2]))));r=play(m.q,r)
   if m.stalled and du:stall.append(i)
   # Left 8T-idler-16T path gives +1/2; right 16T pair gives -1.
   selected=1 if r < -contact else -1 if r>contact else 0
   Qangle+=out.step(1 if selected==1 else -2,selected!=0)
   if abs(lift-cam)<1e-6:roller+=(e.q-old_e)*math.sqrt(1+((cam-oldlift)/(e.q-old_e))**2)/3.6*180/math.pi if abs(e.q-old_e)>1e-9 else 0
   old_e=e.q;oldlift=cam
  lift=max(cam_lift(e.q),0 if pocket_clear(m.q) else 3.2);Q='1' if r < -contact else '0' if r>contact else 'undriven'
  if i%2==0:
   frames.append(dict(t=i/180,q=m.q,qm=m.q,qe=e.q,rm=r,re=er,s=lift,bm=m.b,be=e.b,wm=m.w,we=e.w,gm=m.g,ge=e.g,power=power,data=data,output=Qangle,roller=roller,angle=0,b=0,w=m.w,g=m.g,write_mode=e.mode,memory_mode=m.mode,drive_contact=er>contact,takeup_remaining=gate.remaining,stalled=bool(m.stalled and gate.active),segment=f'D {d0}→{d1} · WRITE {w0}→{w1} · Q {Q} · '+('INPUT BLOCKED' if m.stalled and gate.active else 'HOLD' if lift<.02 else 'unlocked')))
 return dict(initial=[d0,w0,bit],target=[d1,w1],profile=f'{backlash:g}° clutch take-up; axial contact {contact:g} mm',backlash_deg=backlash,contact_shift_mm=contact,frames=frames,stalled_steps=len(stall),worm_constraint_error_mm=max_error,final_Q=Q,final_locked=pocket_clear(m.q) and lift<.02)
cases=[]
for d,w,dd,ww in itertools.product([0,1],repeat=4):
 if (d,w)==(dd,ww):continue
 for bit in ([d] if w else [0,1]):
  for backlash,contact in [(0,1.95),(30,1.95),(60,1.95),(0,.8)]:cases.append(run(d,w,dd,ww,bit,backlash,contact))
report=dict(model='Angle-driven rigid quasi-static contact model',rpm=60,angular_step_deg=2,actuator_equation='dq = pi/360 * (dw - 8*dg), angles in degrees; b selected from native gear/printed lever and carriage contact constraints',profiles='0/30/60 degree clutch pickup plus early axial-contact sensitivity; these are assumptions, not measured minimum/maximum backlash.',limitations=['Rigid/no inertia; illustrative equal 60 rpm motor inputs.','Lever band biased toward neutral; contact grid 0.25 degree gear, 0.1 degree lever, 0.01 mm carriage.','Axial clutch contact plus angular lost motion; no force/chamfer/impact solver or certified dog fit.','Source worm mesh geometry and strength require independent qualification.','A blocked memory drive is reported; motor torque, gear flex and loaded release are not solved.'],cases=cases)
(O/'Coupled operation.json').write_text(json.dumps(report,separators=(',',':'),default=lambda x:x.item()))
print(json.dumps(dict(cases=len(cases),frames=sum(len(c['frames']) for c in cases),max_worm_constraint_error_mm=max(c['worm_constraint_error_mm'] for c in cases),blocked_cases=sum(c['stalled_steps']>0 for c in cases)),indent=2))
