from pathlib import Path
p=Path('work/planar-module-restart/adapt.py');s=p.read_text()
s=s.replace("lowright=rawright^box([-100,-100,-100],[100,100,9.5])","lowright=rawright^box([-100,-100,-100],[100,100,9.5])\nlowleft-=box([-100,12.8,7.2],[100,100,10]);lowright-=box([-100,12.8,7.2],[100,100,10])")
s=s.replace('[b,6,39.8]','[b,6,38.8]').replace('[b,22.4,39.8]','[b,22.4,38.8]')
s=s.replace("front-=cy(2.5,16.3,24.5,2,[32,y,0])", "front+=cy(3.8,24.3,28.4,2,[32,y,0])\n front-=cy(2.5,16.3,28.5,2,[32,y,0])")
s=s.replace("add('Upper actuator cheek',front,bed='top'", "add('Upper actuator cheek',front,bed='bottom'")
s=s.replace("add('Locking bolt',bolt,'bolt','rear'", "add('Locking bolt',bolt,'bolt','front'")
s=s.replace('mm3=vv))','mm3=vv,bounds=(a^b).bounding_box()))')
p.write_text(s)
