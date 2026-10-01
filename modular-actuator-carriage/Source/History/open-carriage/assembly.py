from pathlib import Path
import os,json,numpy as np
R0=Path(__file__).resolve().parents[1];O=R0/'open-carriage-candidate';os.environ['PLANAR_OUTPUT']=str(O)
exec(compile((R0/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
core_names=['Upper actuator cheek','Lower actuator cheek','Actuator lever','U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer','Actuator cheek joining pin 31.4','Actuator cheek joining pin 39.4','Actuator return band']
core=[p for p in parts if p['name'] in core_names];ps={p['name']:p['s'] for p in parts};checks=[]
def trial(label,name,axis,sign,obstacles):
 s=ps[name];worst={}
 for shift in np.linspace(65,0,131):
  v=[0,0,0];v[axis]=sign*float(shift)
  moved=s.translate(v);bb=np.array(moved.bounding_box()).reshape(2,3)
  for other,solid in obstacles:
   ob=np.array(solid.bounding_box()).reshape(2,3)
   if np.any(bb[1]<ob[0]) or np.any(ob[1]<bb[0]):continue
   h=moved^solid;vol=h.volume()
   if vol>worst.get(other,{}).get('volume_mm3',.00001):worst[other]=dict(volume_mm3=vol,shift=sign*float(shift),bounds=h.bounding_box())
 checks.append(dict(step=label,part=name,axis=axis,direction=sign,samples=131,worst_intersections=worst));print(label,worst,flush=True)
obs=[(p['name'],p['s']) for p in core]
# Preload joining pins into the body from its exposed -X joint face.
for n in ['Carriage joining pin 0.5','Carriage joining pin 32']:
 p=next(p for p in parts if p['name']==n);c=p['mesh'].bounds.mean(0);ps[n]=cy(2.45,-9.7,6.3,0,c)+cy(3.2,-2.1,-1.3,0,c)
 trial('Preload joining pin into right half from -X',n,0,-1,[('Carriage body',ps['Carriage body'])])
trial('Right half from -Y with actuator complete and worm positioned','Carriage body',1,-1,obs+[('Worm',ps['U015'])])
for n in ['Carriage joining pin 0.5','Carriage joining pin 32']:trial('Preloaded joining pin follows right half from -Y',n,1,-1,obs)
trial('Left half from -X onto right half and preloaded pins','Carriage bearing end',0,-1,obs+[(n,ps[n]) for n in ['Carriage body','Carriage joining pin 0.5','Carriage joining pin 32','U015']])

for x in [-11.0,11.0]:
 n='Control rod attachment pin '+str(x);ps[n]=cy(2.45,6.6,22.6,1,[x,0,32])+cy(3.2,14.2,15.0,1,[x,0,32])
 trial('Preload rod attachment pin into carriage from -Y',n,1,-1,[(k,ps[k]) for k in ['Carriage body','Carriage bearing end']]+obs)
trial('Control rod from -Y onto preloaded pins','Carriage control rod',1,-1,[(n,ps[n]) for n in ['Carriage body','Carriage bearing end','Control rod attachment pin -11.0','Control rod attachment pin 11.0']]+obs)
passed=all(all(h['volume_mm3']<.02 for h in r['worst_intersections'].values()) for r in checks)
(O/'Complete actuator assembly checks.json').write_text(json.dumps(dict(passed=passed,steps=checks,core=core_names,pose='Carriage centred at X=0; lever at model neutral angle; bolt and bearing walls installed afterwards.',scope='0.5 mm sampled straight insertion paths, not merely fitted operating poses. Includes both cheek joining pins, gear, lever, axles and retainers.'),indent=2));print('PASSED',passed,flush=True)
