"""Front-accessible fixed band pegs with retained roots and shallow end flanges."""
def open_anchors(guide,box,cy):
 for sign in [-1,1]:
  edge=-5.05+sign*6
  root_end=edge+sign*1.2
  xa,xb=(-20,root_end) if sign<0 else (root_end,20)
  guide-=box([xa,19.9,39.7],[xb,30.1,46.7])
  a,b=sorted([edge+sign*.4,edge+sign*3])
  guide+=cy(1.4,a,b,0,[0,27.6,42.8])
  a,b=sorted([edge+sign*2.75,edge+sign*3.55])
  cap=cy(2,a,b,0,[0,27.6,42.8])
  cap=cap.translate([0,-27.6,-42.8]).scale([1,.7,1]).translate([0,27.6,42.8])
  guide+=cap
 return guide
