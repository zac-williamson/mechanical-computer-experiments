"""Wall model transforms, including exact slotted-bellcrank kinematics."""
import json,math,hashlib
from pathlib import Path
import numpy as np
import trimesh
from wall_phases import group
from compact_pose import joint as oldjoint, vertices as oldvertices
ROOT=Path(__file__).resolve().parents[1]
PHASES={}
_phase_file=ROOT/'Wall register/External gear phase checks.json'
if _phase_file.exists():
 _phase_report=json.loads(_phase_file.read_text())
 if _phase_report.get('external_profile_pass') and _phase_report.get('geometry_sha256')==hashlib.sha256((ROOT/'Wall register/geometry.npz').read_bytes()).hexdigest():
  PHASES=_phase_report['shaft_phases_deg']
OLD={p['id']:p for p in json.loads((ROOT/'Compact layout/parts.json').read_text())}
def displacement(p,f):return -f['rail'] if p.get('control_key')=='clock' else f['write']['q']
def local_joint(p,f):
 if 'baseline_id' in p:
  sh,ax,c,an=oldjoint(OLD[p['baseline_id']],f)
  return sh,ax,c+np.array(p['placement_shift']),an
 sh=np.zeros(3);ax=np.array([1.,0,0]);c=np.array(p.get('centre',[0,0,0]),float);an=0.
 motion=p.get('motion','fixed')
 if motion=='crosshead':sh[0]=f['rail']
 elif motion=='write-fork':sh[0]=-f['write']['q']
 elif motion=='control-rod':sh[2]=displacement(p,f)
 elif motion=='bellcrank':ax=np.array([0.,1,0]);c=np.array(p['pivot'],float);an=-math.asin((f['rail'] if p.get('control_key')=='clock' else -f['write']['q'])/p['radius'])
 elif p.get('drive'):
  d=p['drive'];sign=-1 if d.startswith('-') else 1;d=d.lstrip('-')
  an=math.radians(sign*(f['clock']['w'] if d=='CLK' else f['angles'][d])+PHASES.get(group(p),p.get('phase_deg',0)))
 return sh,ax,c,an

def joint(p,f):
 sh,ax,c,an=local_joint(p,f)
 if 'assembly_rotation' in p:
  r=np.array(p['assembly_rotation']);t=np.array(p['assembly_translation'])
  return r@sh,r@ax,r@c+t,an
 return sh,ax,c,an

def vertices(p,v,f):
 if 'baseline_id' in p:
  r=np.array(p.get('assembly_rotation',np.eye(3)));t=np.array(p.get('assembly_translation',[0,0,0]))
  shift=np.array(p['placement_shift']);local=(v-t)@r
  return (oldvertices(OLD[p['baseline_id']],local-shift,f)+shift)@r.T+t
 sh,ax,c,an=joint(p,f);t=trimesh.transformations.rotation_matrix(an,ax,c);t[:3,3]+=sh
 return trimesh.transform_points(v,t)

def example_frames():
 report=json.loads((ROOT/'Compact layout/Compact contact-resolved operation.json').read_text())
 return next(c['frames'] for c in report['cases'] if c['start']==[1,1,0] and c['end']==[1,1,1] and c['initial_Q']==0)

def transform(p,f):
 sh,ax,c,an=joint(p,f);t=trimesh.transformations.rotation_matrix(an,ax,c);t[:3,3]+=sh
 return t
