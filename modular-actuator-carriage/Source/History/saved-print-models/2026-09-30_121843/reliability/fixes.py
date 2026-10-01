"""Local print fixes preserving moving-part interfaces and peg retention in Z."""
def fix_bolt(bolt,box,cy):
 for sign in [-1,1]:
  edge=-5.05+sign*6;a,b=sorted([edge+sign*2.75,edge+sign*3.55])
  cap=cy(2,a,b,0,[0,27.6,49.6])
  printable=cap.translate([0,-27.6,-49.6]).scale([1,.7,1]).translate([0,27.6,49.6])
  bolt-=box([a-.002,25.59,47.59],[b+.002,29.61,51.61])
  sa,sb=sorted([edge+sign*.4,edge+sign*3])
  bolt+=cy(1.4,sa,sb,0,[0,27.6,49.6])+printable
 return bolt
