"""Shared manufactured cam outline and circular-follower contact calculation."""
import math
import numpy as np
from shapely.geometry import LineString

RADIUS=3.6
BASE_CENTRE_Z=53.3
AMPLIFICATION=2.5
def outline():
 pts=[[-88,49.7]]
 for bx,sign in [(-5.05,1),(100.95,-1)]:
  q=np.linspace(-18,18,361)
  lift=3.8*np.clip((sign*q-.6)/3.8,0,1)
  path=np.stack([bx-q,BASE_CENTRE_Z+lift],axis=1)[::-1]
  edge=list(LineString(path).offset_curve(-RADIUS,join_style=1).coords)
  if edge[0][0]>edge[-1][0]:edge.reverse()
  pts.extend(edge)
 pts.append([155,49.7])
 return np.array(pts)

PROFILE=outline()
def follower_height(x):
 """Lowest circle centre above every segment of the actual cam surface."""
 best=-math.inf
 for (a,za),(b,zb) in zip(PROFILE[:-1],PROFILE[1:]):
  if b<=a:raise ValueError('Cam surface must be monotonic in X')
  lo=max(a,x-RADIUS);hi=min(b,x+RADIUS)
  if lo>hi:continue
  slope=(zb-za)/(b-a)
  tangent=x+RADIUS*slope/math.sqrt(1+slope*slope)
  xx=min(hi,max(lo,tangent))
  z=za+slope*(xx-a)+math.sqrt(max(0,RADIUS**2-(xx-x)**2))
  best=max(best,z)
 if not math.isfinite(best):raise ValueError('Follower outside cam')
 return best

def lifts(bar):
 return {bank:max(0.,follower_height(bx-bar)-BASE_CENTRE_Z)
         for bank,bx in [('master',-5.05),('slave',100.95)]}
