from pathlib import Path
import os,json,numpy as np
R0=Path(__file__).resolve().parents[1];O=R0/'neck-candidate';os.environ['PLANAR_OUTPUT']=str(O)
exec(compile((R0/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
core=[p for p in parts if p['name'] in ['Upper actuator cheek','Lower actuator cheek','Actuator lever','U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer']]
print('CORE',[(p['name'],p['s'].bounding_box()) for p in core],flush=True)
for name,sign in [('Carriage body',1),('Carriage bearing end',-1)]:
 s=next(p['s'] for p in parts if p['name']==name)
 for axis in [0,1]:
  hits={}
  for shift in np.linspace(0,65,27):
   v=[0,0,0];v[axis]=sign*shift
   for p in core:
    h=s.translate(v)^p['s']
    if h.volume()>.04:
     # Report obstruction in carriage coordinates.
     key=p['name'];h=h.translate([-u for u in v]);hits[key]=h if key not in hits else hits[key]+h
  print(name,'axis',axis,'sign',sign,[(k,round(h.volume(),1),h.bounding_box()) for k,h in hits.items()],flush=True)
print('HARDWARE',[(p['name'],p['mesh'].bounds.tolist()) for p in parts if 'joining' in p['name']],flush=True)
