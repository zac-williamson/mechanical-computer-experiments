from pathlib import Path
p=Path('work/planar-module-restart/adapt.py');s=p.read_text()
s=s.replace('translate([0,6,-6.3])','translate([0,6,-5.7])').replace('32.5],[2,28.8,39.01]','33.1],[2,28.8,39.61]')
a=s.index('originalbolt=');b=s.index('# Replace guide feet',a)
s=s[:a]+'''originalbolt=source('Direct lock bolt');shift=[0,6,-5.7]
# Retain the source head, axle bore and tip; remove its tall rear band arm.
bolt=originalbolt.translate(shift)^box([-100,-100,-100],[100,31.2,52.2])
guide=source('Detachable bolt guide').translate(shift)^box([-100,-100,-100],[100,100,55.8])
# Paired short side seats keep band force close to the guided head. The
# source's front/rear retaining faces and tip guide remain in place.
for sign in [-1,1]:
 edge=-5.05+sign*6
 lo,hi=sorted([edge-sign*.2,edge+sign*3])
 hook=cy(1.4,lo,hi,0,[0,27.6,49.6])
 lo,hi=sorted([edge+sign*2.75,edge+sign*3.55])
 hook+=cy(2,lo,hi,0,[0,27.6,49.6]);bolt+=hook
 lo,hi=sorted([edge-sign*.1,edge+sign*4])
 guide-=box([lo,25.2,47.2],[hi,30,56])
 lo,hi=sorted([edge+sign*.4,edge+sign*3])
 anchor=cy(1.4,lo,hi,0,[0,27.6,42.8])
 lo,hi=sorted([edge+sign*2.75,edge+sign*3.55])
 anchor+=cy(2,lo,hi,0,[0,27.6,42.8]);guide+=anchor
# The old rear band anchor is not used.
guide-=box([-9,37.3,40.2],[-1.1,42,47.2])
''' +s[b:]
s=s.replace('cz=47+math.sqrt','cz=47.6+math.sqrt')
p.write_text(s)
p=Path('work/planar-module-restart/hardware.py');s=p.read_text().replace('[-5.05,y,47]','[-5.05,y,47.6]').replace('[-5.05,23.6,47]','[-5.05,23.6,47.6]')
a=s.index('lockband=');b=s.index('# Four-stud',a)
s=s[:a]+'''for sign in [-1,1]:
 x=-5.05+sign*8.05
 lockband=loop([27.6,42.8],[27.6,49.6],1.5,2.1,x-.5,x+.5).rotate([90,0,90])
 native(('Left' if sign<0 else 'Right')+' lock return band',mesh(lockband),'lock-band',kind='elastic',color=(.56,.25,.51))
''' +s[b:]
p.write_text(s)
