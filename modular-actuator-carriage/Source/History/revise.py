from pathlib import Path
p=Path('work/planar-module-restart/adapt.py');s=p.read_text()
s=s.replace('box([-100,-100,9.5],[100,18,100])+box([-100,-100,24],[100,100,100])','box([-100,-100,9.5],[100,18,23.5])+box([-100,-100,23.5],[100,14.6,100])')
s=s.replace('[100,23,100])).translate([0,6,-6.3])','[4,23,100])).translate([0,6,-6.3])')
a=s.index('front=fold(');b=s.index('lever=fold(',a)
s=s[:a]+'''front=fold(source('Memory — Front bearing cheek'))^box([-100,-100,-100],[36,100,100])
rear=fold(source('Memory — Rear bearing cheek'))^box([-100,-100,-100],[36,100,100])
# Keep the original +X mounting direction; shorten its remote extension.
front+=box([28,20.4,20.4],[36,34.6,24.4])
rear+=box([28,20.4,7.6],[36,34.6,11.6])
for y in [24.4,31]:
 front-=cy(2.5,16.3,24.5,2,[32,y,0])+cy(3.25,20.3,20.8,2,[32,y,0])
 rear+=box([28,y-3.2,11.5],[36,y+3.2,20.0])
 rear-=cy(2.5,7.5,20.1,2,[32,y,0])+cy(3.25,19.5,20.1,2,[32,y,0])
rear+=box([32,24,7.6],[40,32,24.4])
for z in [12,20]:rear-=cy(2.5,23.9,32.1,1,[36,0,z])+cy(3.25,31.6,32.1,1,[36,0,z])
rear-=front
''' + s[b:]
a=s.index('for isright,a,b');b=s.index("add('Carriage body'",a)
s=s[:a]+'''for isright,a,b in [(False,-15.6,-8),(True,8,15.6)]:
 web=box([a,2,9],[b,6,39.8])+box([a,2,35.8],[b,22.4,39.8])
 web+=box([a,14.4,28.2],[b,38.4,36])
 web-=clear
 x=11.8 if isright else -11.8
 web-=cy(2.5,14.3,22.5,1,[x,0,32])+cy(3.25,14.3,14.8,1,[x,0,32])
 if isright:right+=web
 else:left+=web
left+=box([-15.6,30.4,28],[7.8,38.4,36])+box([-15.6,22.8,35.5],[-8,39.5,39])
# The added webs do not fill the source's two locking pockets.
keeper_void=box([-12,22.8,32.5],[2,28.8,39.01])-keeper
left-=keeper_void
for name in ['left','right']:
 s=locals()[name]
 for y,z in [(24,3.5),(34.4,32)]:s-=cy(2.5,.1,16.3,0,[0,y,z])+cy(3.3,7.8,8.6,0,[0,y,z])
 s-=cy(2.65,-20,20,0,[0,10.2,16])
 locals()[name]=s
''' +s[b:]
s=s.replace('frame+=box([-36,32,z-4],[-28,48.4,z+4]);frame-=cy(2.5,31.9,40.1,1,[-32,0,z])+cy(3.25,31.9,32.4,1,[-32,0,z])','frame+=box([32,32,z-4],[40,48.4,z+4]);frame-=cy(2.5,31.9,40.1,1,[36,0,z])+cy(3.25,31.9,32.4,1,[36,0,z])')
s=s.replace('frame-=rear+guide','frame-=box([-31.1,31.9,7.2],[31.1,44.3,25.4])\nframe-=rear+front+guide')
p.write_text(s)
