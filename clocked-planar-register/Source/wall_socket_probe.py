"""Exclude only the declared 90-degree roof sector from friction-pin probes.

Printed guide cheeks have horizontal frame-pin bores with a 45-degree roof.
The other 270 degrees must retain the complete specified socket wall. Axle
bearings are unaffected and still require their full annular bed-contact face.
"""
import numpy as np,manifold3d as m

def socket_probe(ring,entry,c,lo,hi):
 if entry.get('pin_roof_axis') is None:return ring
 assert entry['pin_roof_axis']==0
 sign=entry['pin_roof_sign'];x,z=c[0],c[2]
 pts=np.array([[x,z],[x+sign*20,z-20],[x+sign*20,z+20]])
 roof=m.CrossSection([pts],m.FillRule.EvenOdd).extrude(hi-lo).transform([[1,0,0,0],[0,0,1,lo],[0,1,0,0]])
 return ring-roof
