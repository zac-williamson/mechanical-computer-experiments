import math
RADIUS=3.6; SLOPE=1.8; CLOSED_Z=57.3; HIGH=61.5
INTERCEPT=CLOSED_Z+SLOPE*41.75-RADIUS*math.sqrt(1+SLOPE*SLOPE)
CREST=(INTERCEPT-HIGH)/SLOPE
TANGENT=CREST+RADIUS*SLOPE/math.sqrt(1+SLOPE*SLOPE)
def cam_lift(q):
 u=38-q
 if u<=CREST:z=HIGH+RADIUS
 elif u<=TANGENT:z=HIGH+math.sqrt(max(0,RADIUS*RADIUS-(u-CREST)**2))
 else:z=INTERCEPT-SLOPE*u+RADIUS*math.sqrt(1+SLOPE*SLOPE)
 return max(0,z-CLOSED_Z)
def bolt_lift(qm,qe):return max(cam_lift(qe),5.4 if abs(qm)<3.3 else 0)
