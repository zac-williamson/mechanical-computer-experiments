"""Geometric mass proxy and force-path audit; not measured actuation force."""
from pathlib import Path
from zipfile import ZipFile
import io,json,hashlib
import numpy as np
import trimesh
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
archive=R.parents[1]/'analysis/wall-register-before-direct-drive/Wall register development package.zip'
baseline_path=O/'Previous moving material.json'
if not baseline_path.exists():
 with ZipFile(archive) as z:
  oldp=json.loads(z.read('Wall register/parts.json'));oldv=np.load(io.BytesIO(z.read('Wall register/geometry.npz')))['vertices'].reshape(-1,3)
p=json.loads((O/'parts.json').read_text());v=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def volumes(parts,vertices):
 result={}
 for x in parts:
  if x['kind']!='printed':continue
  a=vertices[x['offset']//3:x['offset']//3+x['vertices']]
  t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
  result[x['id']]=dict(module=x['module'],motion=x.get('motion','fixed'),volume_mm3=abs(t.volume))
 return result
old=json.loads(baseline_path.read_text()) if baseline_path.exists() else volumes(oldp,oldv)
baseline_path.write_text(json.dumps(old,indent=2))
new=volumes(p,v)
def moving(items,module):return sum(x['volume_mm3'] for x in items.values() if x['module']==module and x['motion']!='fixed')
comparison={}
for module in ['bit','control']:
 a,b=moving(old,module),moving(new,module)
 comparison[module]=dict(before_printed_moving_volume_mm3=a,after_printed_moving_volume_mm3=b,reduction_percent=100*(1-b/a))
# Density is intentionally unspecified; identical material/infill is required for a mass comparison.
result=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,moving_printed_material=comparison,parts=new,
 controller_direction_bellcranks_before=2,controller_direction_bellcranks_after=0,
 added_free_rollers_per_bit=4,write_rod_total_travel_before_mm=11.25,write_rod_total_travel_after_mm=7.5,
 force_model={
  'WRITE_ideal_actuator_force':'N * F_write_selector; unchanged overall ideal mechanical advantage',
  'WRITE_shared_rod_force':'N * F_write_selector, versus (2/3)*N*F_write_selector previously; shorter rod travel trades for higher rod force',
  'CLOCK_ideal_actuator_force':'2.5 * N * F_local_clock_bar; retained 2.5:1 stroke amplifier',
  'actual_force':'Add guide, pivot, rolling resistance and clutch friction, plus inertial force. Values are unmeasured.',
  'roller_assumption':'Bush and axle rotate together in circular crank bores. The viewer prescribes carrier motion; it does not solve free roller spin or prove rolling rather than skid.'},
 minimum_new_bellcrank_web_mm=6,minimum_new_pivot_radial_wall_mm=2,
 limitations=['No load, stiffness, fatigue or friction qualification.','Printed volume is a mass proxy; native hardware and infill are not included.','N=8 multiplies row loads; lower component count does not prove that the shared actuator has adequate force.','Native bush rollers are plain-bearing rollers, not ball bearings.'])
(O/'Force and mass review.json').write_text(json.dumps(result,indent=2))
print(json.dumps(comparison,indent=2))
