"""Absolute-angle clutch contact candidate. Entry-window waits require native surface verification."""
from pathlib import Path
from itertools import product
import json,math,time,hashlib
import numpy as np
from actuator import Actuator,tables
from operation import Clutch,ring,pocket
from compact_cam import lifts,AMPLIFICATION
from compact_phase_clutch import PhaseClutch,clear,flank,WINDOWS
from compact_pose import phases

R=Path(__file__).resolve().parents[1]/'Compact layout'
T=tables()
# Interpolate geometric circle/segment contact for integration efficiency.
BAR=np.linspace(-10,10,4001)
LIFT=np.array([[lifts(float(s))[k] for k in ['master','slave']] for s in BAR])
def run(start,end,q0,slack=None,contact=6.8,steps=3000,keep=False,record_stride=6,warmup=0):
 if slack is not None:raise ValueError("Fixed backlash delays are obsolete; use the native dog angular windows")
 d,w,k=start;dd,ww,kk=end
 selected=lambda d,w,q:d if w else q
 initial_m=selected(d,w,q0) if k==0 else q0
 m=Actuator(2*initial_m-1,T);o=Actuator(1-2*q0,T)
 we=Actuator(1-2*w,T);ck=Actuator(2*k-1,T)
 rm=m.q-math.copysign(.4,m.q);ro=o.q-math.copysign(.4,o.q)
 rw=we.q-math.copysign(.4,we.q)
 gates=[PhaseClutch() for _ in range(5)]
 ph=phases()
 positions=[-AMPLIFICATION*ck.q]*2
 waits=[0,0];face_blocks=[]
 def storage_key(bank,r):return bank+(' L072' if r>.8 else ' L102') if abs(r)>.8 else None
 def storage_gear(bank,key,P):
  sign=(1 if key.endswith('L072') else -1)*(1 if bank=='master' else -1)
  return sign*P+ph[key]
 def selector_key(r):return 'write L102' if r<-.8 else 'write L072' if r>.8 else None
 def selector_gear(key,a):return a['D' if key.endswith('L102') else 'Q']+ph[key]
 mk=storage_key('master',rm);qk=storage_key('slave',ro);xk=selector_key(rw)
 mdirection=2*initial_m-1;qdirection=2*q0-1;xdirection=2*selected(d,w,q0)-1
 def initial_angles(P,D):
  a=dict(D=D,WRITE=0.,CLK=0.,POWER=P,X=0.,M=0.,Q=0.)
  a['M']=storage_gear('master',mk,P)+flank(mk,mdirection)-ph['Master output']
  a['Q']=storage_gear('slave',qk,P)+flank(qk,qdirection)-ph['Q']
  a['X']=selector_gear(xk,a)+flank(xk,xdirection)-ph['Selected data']
  return a
 # Choose an assembly phase satisfying the already engaged input clutch.
 a0=initial_angles(0,0)
 if k==0:
  desired=m.w-flank('master_gate L102',xdirection)-ph['master_gate L102']
  if w:Dphase=desired-a0['X'];Pphase=0
  else:Pphase=(desired-a0['X'])/qdirection;Dphase=0
 else:
  desired=-o.w+flank('slave_gate L102',-mdirection)+ph['slave_gate L102']
  Pphase=(desired-a0['M'])/mdirection;Dphase=0

 angle=initial_angles(Pphase,Dphase)
 lastqd=2*qdirection
 gates[2].key=mk;gates[3].key=qk;gates[4].key=xk
 gates[0 if k==0 else 1].key=('master_gate' if k==0 else 'slave_gate')+' L102'
 stalls=[];conflicts=[];frames=[]
 expected=None if k==0 and kk==1 and (d,w)!=(dd,ww) else selected(d,w,q0) if k==0 and kk==1 else q0
 for i in range(-warmup,steps+1):
  dd,ww,kk=start if i<0 else end
  s=-AMPLIFICATION*ck.q
  cam=np.array([np.interp(s,BAR,LIFT[:,j]) for j in range(2)])
  # A bolt above an unaligned pocket rests on the carriage roof.
  actual=[max(cam[j],0 if pocket(a.q) else 3.2) for j,a in enumerate([m,o])]
  qbit=1 if lastqd>1e-8 else 0 if lastqd<-1e-8 else None
  if keep and i>=0 and i%record_stride==0:
   frames.append(dict(turns=i/180,master=m.state(),slave=o.state(),write=we.state(),clock=ck.state(),
    rm=rm,ro=ro,rw=rw,rail=s,master_gate_lag=positions[0]-s,slave_gate_lag=positions[1]-s,master_lift=actual[0],slave_lift=actual[1],Q=qbit,Q_rate_ratio=lastqd/2,angles=dict(angle)))
  if i==steps:break
  old=dict(angle)
  angle['D']+=2*(2*dd-1);angle['WRITE']+=2*(2*ww-1);angle['CLK']+=2*(2*kk-1);angle['POWER']+=2
  we.step(-2*(2*ww-1));ck.step(2*(2*kk-1));rw=ring(we.q,rw)
  for name,act in [('WRITE',we),('CLK',ck)]:
   if act.stalled:stalls.append((i,name,act.q))
  for j,bank,r,out,group in [(2,'master',rm,'M','Master output'),(3,'slave',ro,'Q','Q')]:
   key=storage_key(bank,r)
   if key:
    du=gates[j].step(key,storage_gear(bank,key,old['POWER']),storage_gear(bank,key,angle['POWER']),angle[out]+ph[group])
    angle[out]+=du
    if gates[j].blocked:face_blocks.append((i,bank))
   else:gates[j].key=None
  key=selector_key(rw)
  if key:
   angle['X']+=gates[4].step(key,selector_gear(key,old),selector_gear(key,angle),angle['X']+ph['Selected data'])
   if gates[4].blocked:face_blocks.append((i,'write'))
  else:gates[4].key=None
  md=angle['M']-old['M'];xd=angle['X']-old['X'];lastqd=angle['Q']-old['Q']
  s=-AMPLIFICATION*ck.q
  for j,(name,a,delta) in enumerate([('master',m,xd),('slave',o,-md)]):
   camlift=float(np.interp(s,BAR,LIFT[:,j]))
   lift=max(camlift,0 if pocket(a.q) else 3.2)
   bank=name+'_gate';key=bank+' L102';sign=1 if j==0 else -1
   gearold=(old['X'] if j==0 else -old['M'])+ph[key]
   gearnew=(angle['X'] if j==0 else -angle['M'])+ph[key]
   active=sign*s>contact
   waiting=active and gates[j].key is None and not clear(key,a.w-gearnew)
   positions[j]=sign*min(sign*s,contact-.01) if waiting else s
   if waiting:waits[j]+=1
   du=gates[j].step(key if active and not waiting else None,gearold,gearnew,a.w)
   if active and lift<3.2-1e-6:conflicts.append((i,name))
   if du:
    a.step(du,locked=lift<3.2-1e-6)
    if a.stalled:stalls.append((i,name,a.q))
  rm=ring(m.q,rm);ro=ring(o.q,ro)
 qfinal=1 if lastqd>1e-8 else 0 if lastqd<-1e-8 else None
 result=dict(start=start,end=end,initial_Q=q0,expected_Q=expected,final_Q=qfinal,
  wrong_defined_output=expected is not None and qfinal!=expected,final_Q_rate_ratio=lastqd/2,
  blocked_steps=len(stalls),blocked_actuators=sorted(set(s[1] for s in stalls)),drive_against_inserted_bolt=len(conflicts),
  master_in_pocket=pocket(m.q),slave_in_pocket=pocket(o.q),
  face_wait_steps=sum(waits),conservative_entry_wait_steps=len(face_blocks),entry_wait_examples=face_blocks[:10],warmup_steps=warmup,dog_free_play_degrees_range=[min(v['upper_deg']-v['lower_deg'] for v in WINDOWS.values()),max(v['upper_deg']-v['lower_deg'] for v in WINDOWS.values())],first_gate_contact_mm=contact,undefined_setup_hold=expected is None)
 if keep:result['frames']=frames
 return result

if __name__=='__main__':
 cases=[];begin=time.monotonic()
 for start in product([0,1],repeat=3):
  for end in product([0,1],repeat=3):
   if start==end:continue
   for q in [0,1]:cases.append(run(start,end,q,keep=True))
  print(start,round(time.monotonic()-begin,1),flush=True)
 report=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),cases=cases,conservative_entry_wait_cases=sum(c['conservative_entry_wait_steps']>0 for c in cases),wrong_defined_outputs=sum(c['wrong_defined_output'] for c in cases),
  blocked_cases=sum(c['blocked_steps']>0 for c in cases),bolt_conflict_cases=sum(c['drive_against_inserted_bolt']>0 for c in cases),
  assumptions=['Native four-dog angular windows determine take-up; fixed backlash delays are rejected','Nominal gate tip overlap at 6.8 mm from the CAD; manufacturing variation remains to be measured',
   'Native tip-profile waiting, with instantaneous spring insertion once aligned',
   'No shaft inertia, measured elastic force, friction or motor acceleration',
   '3D dog surface clearance is checked separately; this report alone is not a clearance certificate'],mechanically_qualified=False)
 (R/'Compact phase-driven operation.json').write_text(json.dumps(report,separators=(',',':'),default=lambda v:v.item()))
 print({k:v for k,v in report.items() if k not in ['cases','assumptions']})
