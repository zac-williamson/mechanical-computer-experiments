"""Quasi-static gate insertion with phase-dependent face waiting.

Uses the diagnostic native axial-tip projection. No impact/friction model.
The fork can retreat four mm; a hard shoulder guarantees withdrawal.
"""
import json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]/'Compact layout'
r=json.loads((R/'Clutch face phase study.json').read_text())
clear=set(next(x for x in r['profiles'] if x['overlap_depth_mm']==1.)['clear_phases_deg'])
def face_clear(angle):
 i=int(round((angle%360)*2))%720
 # Require adjacent samples clear as well; do not interpret one sample as
 # clearance over the complete half-degree cell.
 return all(((j%720)/2) in clear for j in [i-1,i,i+1])
class GateFork:
 def __init__(self,sign,bar):
  self.sign=sign;self.position=bar;self.engaged=sign*bar>7.8;self.waits=0
 def step(self,bar,relative_angle):
  s=self.sign
  if s*bar<=6.8:self.engaged=False
  waiting=s*bar>7.3 and not self.engaged and not face_clear(relative_angle)
  self.position=s*min(s*bar,7.3) if waiting else bar
  if not waiting and s*bar>7.8:self.engaged=True
  if waiting:self.waits+=1
  lag=self.position-bar
  if not -4-1e-9<=s*lag<=1e-9:raise ValueError(('fork travel exceeded',bar,self.position))
  return self.position,self.engaged,waiting
