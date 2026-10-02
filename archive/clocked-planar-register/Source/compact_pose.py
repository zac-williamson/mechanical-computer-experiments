"""Compact world transforms shared by motion inspection and geometry audits.

Angles follow the quasi-static actuator trace; gear tooth assembly phases and
friction are not established by these transforms.
"""
import math
import numpy as np
import trimesh

ROOT={'master':(0,0,1),'slave':(106,0,1),'write':(-76,-16,-1),'clock':(40,-16,-1)}
def raw_joint(p,f):
 name=p['id'];motion=p.get('motion','fixed');bank=name.split()[0]
 shift=np.zeros(3);axis=np.array([1.,0,0]);centre=np.array(p.get('centre',[0,0,0]),dtype=float);angle=0.
 a=f['angles']
 if motion=='shaft-retainer':
  group=p['drive_group']
  if group in ['master worm','slave worm']:angle=f[group.split()[0]]['w']
  elif group=='clock worm':angle=-f['clock']['w']
  elif group=='write worm':angle=-f['write']['w']
  elif group=='CLK input':angle=f['clock']['w']
  else:angle={'D input':a['D'],'D inverted':-a['D'],'master idler':a['POWER'],'Q inverted':-a['Q'],'POWER input':a['POWER'],'Selected data':a['X']}[group]
  return shift,axis,centre,math.radians(angle)

 if motion=='gate-fork':
  shift[0]=f['rail']+f.get(bank+'_lag',0.);return shift,axis,centre,0.
 if motion=='crosshead' or 'crosshead pin' in name or name.startswith('Crosshead post pin'):
  shift[0]=f['rail'];return shift,axis,centre,0.
 if motion=='amplifier' or name.startswith(('Clock amplifier input','Clock amplifier output','Clock input bush','Clock output bush')):
  return shift,np.array([0.,1,0]),np.array([40.,0,-56]),math.asin(f['rail']/30)
 if motion=='bolt' or 'roller axle' in name or 'roller bush' in name:
  shift[2]=f[bank+'_lift'];return shift,axis,centre,0.
 if bank in ['master_gate','slave_gate']:
  stage='master' if bank=='master_gate' else 'slave';gx=-60 if stage=='master' else 60
  centre=np.array([gx,10.2,16.])
  if 'L099' in name:shift[0]=f['rail']+f.get(bank+'_lag',0.)
  if 'L102' in name:angle=a['X'] if stage=='master' else -a['M']
  elif 'B-input' in name:
   centre[2]=0;angle=-a['X'] if stage=='master' else a['M']
  else:angle=f[stage]['w']
  return shift,axis,centre,math.radians(angle)
 if bank in ROOT and p.get('source'):
  x,z,sign=ROOT[bank];state=f[bank];centre=np.array([x,10.2,z],dtype=float)
  if motion in ['carriage','worm']:shift[0]=sign*state['q']
  if motion=='clutch-ring':shift[0]=sign*f[{'master':'rm','slave':'ro','write':'rw'}[bank]]
  if motion in ['rocker','lever']:
   return shift,np.array([0.,1,0]),np.array([x+sign*13.192323604,10.2,z+sign*32.128448698]),math.radians(state['b'])
  if motion=='gear':
   centre[2]+=sign*24;axis=np.array([0.,1,0]);angle=state['g']
  elif motion=='worm':centre[2]+=sign*16;angle=sign*state['w']
  elif motion in ['O-route','clutch-ring']:angle=a[{'master':'M','slave':'Q','write':'X'}[bank]]
  elif motion in ['A','B']:
   if bank=='write':angle=a['Q' if motion=='A' else 'D']
   else:angle=a['POWER']*(1 if (motion=='A')==(bank=='master') else -1)
  return shift,axis,centre,math.radians(angle)
 if motion=='carriage' and bank in ROOT:
  shift[0]=ROOT[bank][2]*f[bank]['q'];return shift,axis,centre,0.
 # Explicit shaft groups. Every new native transmission part must be mapped.
 if p['kind']=='native' and p.get('axis')==0 and p.get('lego_part')!='2780':
  if name.startswith(('master POWER','slave POWER','POWER shaft')):angle=-a['POWER']
  elif 'reversing idler' in name or 'idler axle' in name:angle=a['POWER']
  elif name.startswith('master output'):angle=a['M']
  elif name.startswith('slave output') or name=='Q takeoff gear':angle=a['Q']
  elif name.startswith('Q feedback'):angle=-a['Q']
  elif name.startswith(('POWER incoming','POWER header')):angle=a['POWER']
  elif name.startswith(('CLK incoming','CLK header gear 0')):angle=a['CLK']
  elif name in ['CLK input axle','CLK header gear 2']:angle=-a['CLK']
  elif name=='WRITE input axle':angle=a['WRITE']
  elif name.startswith('WRITE selector') or name=='Selected data route 16T':angle=a['X']
  elif name=='Master gate data stub 2L':angle=-a['X']
  elif name in ['WRITE D input gear','WRITE D input axle 4L','D header output 8T']:angle=-a['D']
  elif name in ['D header input 8T','D external input 4L']:angle=a['D']
  else:raise ValueError('Unmapped rotating part: '+name)
 return shift,axis,centre,math.radians(angle)

def shaft_group(p):
 n=p['id']
 if p.get('motion')=='shaft-retainer':return p['drive_group']
 if p['kind']!='native':return None
 if 'POWER 16T' in n or n.startswith('POWER shaft'):return 'POWER internal'
 if 'reversing idler' in n or 'idler axle' in n:return n.split()[0]+' idler'
 if n.startswith(('POWER header','POWER incoming')):return 'POWER input'
 if n in ['D header input 8T','D external input 4L']:return 'D input'
 if n in ['D header output 8T','WRITE D input gear','WRITE D input axle 4L']:return 'D inverted'
 if n.startswith('Q feedback'):return 'Q inverted'
 if n=='Q takeoff gear' or n.startswith('slave output') or n in ['slave L097','slave L099','slave L069']:return 'Q'
 if n=='slave_gate B-input' or n.startswith('master output') or n in ['master L097','master L099','master L069']:return 'Master output'
 if n=='Selected data route 16T' or n.startswith('WRITE selector output') or n in ['write L097','write L099','write L069','write L105']:return 'Selected data'
 if n in ['Master gate data stub 2L','master_gate B-input']:return 'Selected data inverted'
 if n.endswith((' L072',' L102')):return n
 return None

from functools import lru_cache
@lru_cache(maxsize=1)
def phases():
 import json,hashlib
 from pathlib import Path
 root=Path(__file__).resolve().parents[1]/'Compact layout'
 report=json.loads((root/'External gear phase checks.json').read_text())
 digest=hashlib.sha256((root/'geometry.npz').read_bytes()).hexdigest()
 if report['geometry_sha256']!=digest:raise ValueError('External gear phases are stale; rerun check_compact_gear_phases.py')
 if not report['external_profile_pass']:raise ValueError('External gear phase check has unresolved collisions')
 return report['shaft_phases_deg']

def joint(p,f):
 sh,axis,centre,angle=raw_joint(p,f);n=p['id'];phase=phases()
 # Keep the worm's original phase and shift the input shaft's arbitrary
 # initial angle to satisfy the external clock mesh. Rotation rates do not change.
 if n.startswith(('CLK incoming','CLK header gear 0')):
  angle=math.radians(f['clock']['w']+phase['CLK input']+phase['CLK worm'])
 elif n in ['CLK input axle','CLK header gear 2']:angle=math.radians(-f['clock']['w'])
 elif n=='WRITE input axle':angle=math.radians(-f['write']['w'])
 else:
  group=shaft_group(p)
  if group in phase:angle+=math.radians(phase[group])
 return sh,axis,centre,angle

def transform(p,f):
 shift,axis,centre,angle=joint(p,f)
 t=trimesh.transformations.rotation_matrix(angle,axis,centre);t[:3,3]+=shift
 return t

def vertices(p,v,f):
 if p.get('motion')=='fork-band':
  from compact_elastic import fork_band
  return fork_band(p['bank'],p['fork_x'],f['rail'],f.get(p['bank']+'_lag',0.))
 if p.get('motion')=='actuator-band':
  from compact_elastic import actuator_band
  return actuator_band(p['bank'],f[p['bank']]['b'])
 if p.get('motion')=='elastic':
  # Lower wrap stays fixed; upper wrap follows the bolt. Straight spans
  # stretch between their tangent heights. Elastic preload is unmeasured.
  out=v.copy();out[:,2]+=np.clip((v[:,2]-50)/8.6,0,1)*f[p['bank']+'_lift'];return out
 return trimesh.transform_points(v,transform(p,f))
