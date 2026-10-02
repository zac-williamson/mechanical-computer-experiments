"""Rigid, unloaded dog-clutch angular constraint derived from native tip faces.

Tracks absolute angles and the actual free angular interval. Invalid axial
insertion is reported, never repaired by teleporting the driven shaft.
"""
import math,json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parents[1]/'Compact layout'
window_report=json.loads((R/'Clutch angular windows.json').read_text())
assert window_report['geometry_sha256']==hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(), 'Regenerate native clutch windows for this geometry'
WINDOWS=window_report['interfaces']
# LDraw vertices are float32 and triangulated independently. A 0.01 degree
# numerical contact gap (~0.001 mm at a dog) avoids classifying coincident
# flanks as penetration. It is NOT a manufacturing-clearance allowance.
CONTACT_GAP_DEG=.01
for w in WINDOWS.values():
 w['lower_deg']+=CONTACT_GAP_DEG;w['upper_deg']-=CONTACT_GAP_DEG
def interval(key,relative):
 w=WINDOWS[key];lo=w['lower_deg'];hi=w['upper_deg'];period=w['period_deg']
 k=math.floor((relative-lo)/period)
 return lo+k*period,hi+k*period

def clear(key,relative,tolerance=1e-7):
 lo,hi=interval(key,relative+tolerance)
 return lo-tolerance<=relative<=hi+tolerance

def flank(key,direction):
 w=WINDOWS[key]
 return w['lower_deg'] if direction>0 else w['upper_deg']

class PhaseClutch:
 def __init__(self):self.key=None;self.blocked=False
 def step(self,key,gear_old,gear_new,output):
  self.blocked=False
  if key is None:self.key=None;return 0.
  relative=output-gear_old
  if self.key!=key:
   # A newly inserted ring must fit the actual current tooth phase.
   if not clear(key,output-gear_new):
    self.key=None;self.blocked=True;return 0.
   self.key=key
   return 0.
  if not clear(key,relative):raise ValueError(('Existing dog penetration',key,relative))
  lo,hi=interval(key,relative+1e-7)
  target=output-gear_new
  return min(hi,max(lo,target))+gear_new-output
