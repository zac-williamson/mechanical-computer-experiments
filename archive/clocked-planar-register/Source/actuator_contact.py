"""Table predictor followed by an exact-profile contact correction.

Corrections preserve q + lead*teeth*g/360, so the commanded worm rotation is
neither invented nor discarded. The predictor chooses the contact branch;
the corrector resolves its finite table quantisation. Force/stability and
continuous swept clearance are separate checks.
"""
from functools import lru_cache
from actuator import Actuator as Predictor,LEAD,TEETH
from contact_projection import correct
@lru_cache(maxsize=65536)
def projection(q,g,b):return correct(q,g,b)
class Actuator(Predictor):
 def __init__(self,side=1,table=None):
  super().__init__(side,table);self.contact_failure=None;self.maximum_position_correction=0.;self.correct_contact()
 def correct_contact(self):
  oldq,oldg,oldb=self.q,self.g,self.b
  r=projection(oldq,oldg%45,oldb)
  if r.get('failed'):
   self.contact_failure=dict(q=oldq,g=oldg,b=oldb,areas=r['areas']);self.stalled=True;return False
  self.q=r['q'];self.g=oldg+r.get('delta_g_deg',0.);self.b=r['b']
  residual=(self.q-oldq)+LEAD*TEETH*(self.g-oldg)/360
  if abs(residual)>1e-10:raise ValueError('Contact projection changed commanded worm lead')
  self.maximum_position_correction=max(self.maximum_position_correction,abs(self.q-oldq));return True
 def step(self,dw,locked=False):
  import numpy as np
  self.stalled=False;self.contact_failure=None
  # Predict reaction rotation first, using the table only to choose a nearby
  # spring-return branch. Nonpenetration and lead conservation decide travel.
  ng=self.g+dw/TEETH
  valid=self.allowed(self.q,ng);ids=np.flatnonzero(valid & (abs(self.t['bs']-self.b)<=1.01))
  nb=float(self.t['bs'][ids[np.argmin(abs(self.t['bs'][ids]))]]) if len(ids) else self.b
  r=projection(self.q,ng%45,nb)
  nq=r['q'];gg=ng+r.get('delta_g_deg',0.)
  if r.get('failed') or abs(nq)>4.45 or (locked and abs(nq-self.q)>1e-9):
   self.stalled=True;self.contact_failure=dict(q=self.q,g=self.g,b=self.b,candidate=r);self.mode='blocked: no feasible contact continuation';return 0.
  residual=nq-self.q+LEAD*TEETH*(gg-self.g)/360-LEAD*dw/360
  if abs(residual)>1e-10:raise ValueError('Integration changed worm lead')
  self.mode='contact-constrained carriage travel' if abs(nq-self.q)>1e-6 else 'reaction gear rotating / carriage stationary'
  self.maximum_position_correction=max(self.maximum_position_correction,abs(nq-self.q))
  self.q=nq;self.g=gg;self.b=r['b'];self.w+=dw
  return dw
 def state(self):
  s=super().state();s['contact_correction_max_mm']=self.maximum_position_correction
  if self.contact_failure:s['contact_failure']=self.contact_failure
  return s
