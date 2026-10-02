"""Check animated angular velocities against every CAD-discovered external mesh."""
from pathlib import Path
import json,copy,hashlib
from compact_pose import joint
R=Path(__file__).resolve().parents[1]/'Compact layout'
ps={p['id']:p for p in json.loads((R/'parts.json').read_text())}
r=json.loads((R/'External gear phase checks.json').read_text())
f=dict(rail=0,rm=0,ro=0,rw=0,master_lift=4,slave_lift=4,angles={k:0. for k in ['D','WRITE','CLK','POWER','X','M','Q']})
for b in ['master','slave','write','clock']:f[b]=dict(q=0,g=0,b=0,w=0)
rows=[]
for driver in ['D','WRITE','CLK','POWER','X','M','Q']:
 h=copy.deepcopy(f);h['angles'][driver]=1
 if driver=='CLK':h['clock']['w']=1
 if driver=='WRITE':h['write']['w']=-1
 for e in r['meshes']:
  va=joint(ps[e['a']],h)[3]-joint(ps[e['a']],f)[3]
  vb=joint(ps[e['b']],h)[3]-joint(ps[e['b']],f)[3]
  error=abs(vb-e['ratio']*va)
  rows.append(dict(driver=driver,a=e['a'],b=e['b'],error_rad=error))
fail=[x for x in rows if x['error_rad']>1e-9]
out=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),mesh_driver_checks=len(rows),failures=fail,rotation_paths_pass=not fail)
(R/'Rotation path checks.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
