import math
BX=-5.05;RADIUS=3.6;SLOPE=1.2;CLOSED_Z=53.3;HIGH=55.1
INTERCEPT=CLOSED_Z+SLOPE*(BX+3.75)-RADIUS*math.sqrt(1+SLOPE*SLOPE)
CREST=(INTERCEPT-HIGH)/SLOPE;TANGENT=CREST+RADIUS*SLOPE/math.sqrt(1+SLOPE*SLOPE)
def cam_lift(q):
 u=BX-q
 z=HIGH+RADIUS if u<=CREST else HIGH+math.sqrt(max(0,RADIUS*RADIUS-(u-CREST)**2)) if u<=TANGENT else INTERCEPT-SLOPE*u+RADIUS*math.sqrt(1+SLOPE*SLOPE)
 return max(0,z-CLOSED_Z)
def bolt_lift(qm,qe):return max(cam_lift(qe),3.2 if abs(qm)<3.0 else 0)
