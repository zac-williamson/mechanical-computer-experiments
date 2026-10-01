from pathlib import Path
p=Path('work/planar-module-restart/adapt.py');s=p.read_text();a=s.index('front=fold(');b=s.index('lever=fold(',a)
s=s[:a]+'''front=fold(source('Memory — Front bearing cheek'))^box([-100,-100,-100],[36,100,100])
rawrear=source('Memory — Rear bearing cheek')
rear=fold(rawrear)^box([-100,-100,-100],[36,100,100])
# Keep the original band seat, shifted 0.6 mm toward -X to stay within the base.
rear+=fold((rawrear^box([33.7,5.9,24.6],[40.7,18.7,31.7])).translate([-.6,0,0]))
front+=box([30.6,27.6,20.4],[38.2,42.4,24.4])
rear+=box([30.6,27.6,7.6],[38.2,42.4,11.6])
for y in [31.4,39.4]:
 front+=cy(3.8,24.3,28.4,2,[34.4,y,0])
 front-=cy(2.5,16.3,28.5,2,[34.4,y,0])+cy(3.25,20.3,20.8,2,[34.4,y,0])
 rear+=cy(3.8,11.5,20.0,2,[34.4,y,0])
 rear-=cy(2.5,7.5,20.1,2,[34.4,y,0])+cy(3.25,19.5,20.1,2,[34.4,y,0])
rear+=box([32,32.4,4],[40,40.4,40])
for z in [8,36]:rear-=cy(2.5,32.3,40.5,1,[36,0,z])+cy(3.25,39.9,40.5,1,[36,0,z])
rear-=front
''' +s[b:]
a=s.index('for z in [12,20]:',s.index('frame=box'));b=s.index('for x in [-28,28]:',a)
s=s[:a]+'''for z in [8,36]:
 frame+=box([32,40.4,z-4],[40,48.4,z+4]);frame-=cy(2.5,40.3,48.5,1,[36,0,z])+cy(3.25,40.3,40.8,1,[36,0,z])
''' +s[b:]
s=s.replace("meta=[];arrays=[]\nfor p in parts:","exec(compile((ROOT/'work/planar-module-restart/hardware.py').read_text(),str(ROOT/'work/planar-module-restart/hardware.py'),'exec'))\nmeta=[];arrays=[]\nfor p in parts:")
s=s.replace("kind=p['kind'],color=p['color'],offset=", "kind=p['kind'],color=p['color'],axis=p.get('axis'),mates=p.get('mates',[]),bed=p.get('bed'),offset=")
p.write_text(s)
