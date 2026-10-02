"""Resolve the recorded table-predictor poses against actual contact profiles.

The worm angle is unchanged; small reaction-angle/carriage corrections retain
its lead constraint. Fork offsets are retained. Clock bar and follower heights
are recomputed from the corrected clock carriage. This is a kinematic trace,
not a force/dynamic solver. Every failed correction stays a failure.
"""
from pathlib import Path
import json,hashlib,time
from copy import deepcopy
from functools import lru_cache
from contact_projection import correct
from actuator import LEAD,TEETH
from compact_cam import lifts,AMPLIFICATION
from operation import pocket
R=Path(__file__).resolve().parents[1]/'Compact layout'
geometry_digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
source=R/'Compact phase-driven operation.json';source_bytes=source.read_bytes();report=json.loads(source_bytes);start=time.monotonic()
assert report.get('geometry_sha256')==geometry_digest, 'Regenerate the operation trace for this geometry'
cachepath=R/'Contact pose cache.json'
solver_digest=hashlib.sha256((Path(__file__).parent/'contact_projection.py').read_bytes()).hexdigest()
persist=json.loads(cachepath.read_text()) if cachepath.exists() else {}
cache=persist.get('poses',{}) if persist.get('solver_sha256')==solver_digest else {}
@lru_cache(maxsize=32768)
def solve(q,g,b):
 key=json.dumps([q,g,b],separators=(',',':'))
 if key not in cache:cache[key]=correct(q,g,b)
 return cache[key]
failures=[];maxdq=0.;maxdg=0.;maxdb=0.;flags=[]
for ci,c in enumerate(report['cases']):
 for fi,f in enumerate(c['frames']):
  oldrail=f['rail']
  for bank,ring in [('master','rm'),('slave','ro'),('write','rw'),('clock',None)]:
   s=f[bank];oldq=s['q'];result=solve(oldq,s['g']%45,s['b'])
   if result.get('failed'):
    failures.append(dict(case=ci,frame=fi,bank=bank,areas=result['areas']));continue
   dg=result.get('delta_g_deg',0.);dq=result['q']-oldq
   assert abs(dq+LEAD*TEETH*dg/360)<1e-10
   s['q']=result['q'];s['g']+=dg;s['b']=result['b'];s['contact_profile_overlap_mm2']=result['areas']
   maxdq=max(maxdq,abs(dq));maxdg=max(maxdg,abs(dg));maxdb=max(maxdb,abs(result.get('delta_b_deg',0.)))
   if ring:
    old=f[ring];f[ring]+=dq
    if (old>.8)!=(f[ring]>.8) or (old<-.8)!=(f[ring]<-.8):flags.append(dict(case=ci,frame=fi,interface=ring))
  f['rail']=-AMPLIFICATION*f['clock']['q'];ll=lifts(f['rail'])
  for bank in ['master','slave']:f[bank+'_lift']=max(ll[bank],0 if pocket(f[bank]['q']) else 3.2)
  for gate,sign in [('master_gate',1),('slave_gate',-1)]:
   lag=f.get(gate+'_lag',0.)
   # During a dog-face wait the ring is arrested by the gear, not by the
   # bar. A small corrected bar position changes lost motion, not the
   # arrested ring position. Preserve that independent contact constraint.
   newlag=oldrail+lag-f['rail'] if abs(lag)>1e-8 else lag
   f[gate+'_lag']=newlag
   if not -4-1e-8<=sign*newlag<=1e-8:flags.append(dict(case=ci,frame=fi,interface=gate+'_float_limit'))
   if (sign*(oldrail+lag)>6.8)!=(sign*(f['rail']+newlag)>6.8):flags.append(dict(case=ci,frame=fi,interface=gate))
 if ci%8==0:print(ci,solve.cache_info(),'seconds',time.monotonic()-start,flush=True)
report.update(scope=__doc__,geometry_sha256=geometry_digest,predictor_sha256=hashlib.sha256(source_bytes).hexdigest(),contact_projection=dict(failures=failures,changed_engagement_classifications=flags,max_carriage_correction_mm=maxdq,max_reaction_correction_deg=maxdg,max_lever_correction_deg=maxdb,unique_poses=solve.cache_info().misses),contact_pose_pass=not failures and not flags,mechanically_qualified=False)
(R/'Compact contact-resolved operation.json').write_text(json.dumps(report,separators=(',',':')))
cachepath.write_text(json.dumps(dict(solver_sha256=solver_digest,poses=cache),separators=(',',':')))
print(json.dumps(report['contact_projection'],indent=2),flush=True)
