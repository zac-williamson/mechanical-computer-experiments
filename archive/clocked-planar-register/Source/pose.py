"""One transform definition shared by clearance checking and the viewer."""
import math,numpy as np,trimesh
# Initial phases from the native tooth-profile sweep. Whole routing shafts
# share each phase; dog engagement is still a separate qualification item.
DATA_PHASE=[0.,5.5,11.5,16.5,.5,5.,12.]
RETURN_PHASE=[0.,11.25,0.,11.25,0.]

def joint(p,f):
 name=p['id'];bank=p['bank'];mo=p['motion'];shift=[0.,0.,0.];axis=[1,0,0];centre=[0.,10.2,0.];angle=0.
 a=f['angles']
 if mo=='rail':shift[0]=f['rail'];return shift,axis,centre,angle
 if mo=='amplifier':return shift,[0,1,0],[75,0,138],-math.asin(f['clock']['q']/20)
 if bank in ['master_gate','slave_gate']:
  stage='master' if bank=='master_gate' else 'slave';gx=-85 if stage=='master' else 75
  q=f[stage+'_ring'];centre=[gx,10.2,16]
  if mo=='bellcrank':return shift,[0,1,0],[gx-10,0,35],math.asin(q/12)
  if mo in ['fork','clutch-ring']:shift[0]=q
  if p['kind']=='native':
   if mo in ['O','O-route','clutch-ring']:angle=f[stage]['w']
   elif mo=='B':angle=-a['X' if stage=='master' else 'M'] if 'L102' in name else a['X' if stage=='master' else 'M'];centre=[gx,10.2,16 if 'L102' in name else 0]
  return shift,axis,centre,math.radians(angle)
 if bank in ['master','slave','write','clock']:
  dx,dz={'master':(0,0),'slave':(160,0),'write':(-85,82.4),'clock':(75,82.4)}[bank]
  state=f[bank];centre=[dx,10.2,dz]
  if mo in ['carriage','worm']:shift[0]=state['q']
  if mo=='clutch-ring':shift[0]=f['rm' if bank=='master' else 'ro' if bank=='slave' else 'rw']
  if mo=='bolt':shift[2]=f[bank+'_lift']
  if mo in ['rocker','lever']:return shift,[0,1,0],[dx+13.192323604,10.2,dz+32.128448698],math.radians(state['b'])
  if p['kind']=='native':
   if mo in ['input','worm']:centre=[dx,10.2,dz+16];angle=state['w']
   elif mo=='gear':centre=[dx,10.2,dz+24];axis=[0,1,0];angle=state['g']
   elif mo in ['O','O-route','clutch-ring']:angle=a['M' if bank=='master' else 'Q' if bank=='slave' else 'X']+(12 if bank=='write' else 0)
   elif mo in ['A','B','A-idler','B-idler']:
    if bank in ['master','slave']:
     if 'L072' in name:angle=-a['POWER']
     elif 'L102' in name:angle=.5*a['POWER']
     else:
      centre=[dx,16.0094750193 if 'idler' in name else 10.2,dz-10.5 if 'idler' in name else dz-16]
      angle=-a['POWER'] if 'idler' in name else a['POWER']
    else:
     input_a=mo.startswith('A');angle=a['Q' if input_a else 'D']*(-2 if 'idler' in name else 1)
     centre=[dx,17.90454381 if 'idler' in name else 10.2,dz if 'L072' in name or 'L102' in name else dz-9.2 if 'idler' in name else dz-18.4]
  return shift,axis,centre,math.radians(angle)
 if bank=='power':return shift,axis,[80,10.2,-16],math.radians(a['POWER'])
 if bank=='transfer':return shift,axis,[47,10.2,0],math.radians(a['M'])
 if bank=='feedback':
  centre=p.get('centre',[0,10.2,64]);index=int(name.split()[-1]) if 'gear' in name or 'axle' in name else int(name.split()[-2]) if 'retainer' in name else 4
  return shift,axis,centre,math.radians(a['Q']*((-1)**index)+RETURN_PHASE[index])
 if bank=='data_route':
  index=int(name.split()[-2]) if 'retainer' in name else int(name.split()[-1]);return shift,axis,p['centre'],math.radians(a['X']*((-1)**index)+DATA_PHASE[index])
 if bank=='write_port':
  centre=p.get('centre',[0,10.2,114.4]);return shift,axis,centre,math.radians(f['write']['w'] if centre[2]<110 else -f['write']['w']+11.25)
 return shift,axis,centre,angle

def transform(p,f):
 shift,axis,centre,angle=joint(p,f)
 t=trimesh.transformations.rotation_matrix(angle,axis,centre);t[:3,3]+=shift
 return t
