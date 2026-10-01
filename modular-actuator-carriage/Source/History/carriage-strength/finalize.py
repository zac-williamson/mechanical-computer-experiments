import os
from pathlib import Path
import json,runpy,shutil
R=Path(__file__).resolve().parent;P=R.parent;O=Path(os.environ.get('PLANAR_OUTPUT',str(P/'adapted')))
for s in [R/'probes.py',R/'functional_check.py']:runpy.run_path(str(s),run_name='__main__')
for f in ['Clearance checks.json','Added carriage material checks.json']:
 d=json.loads((O/f).read_text());assert not any(d[k] for k in ['printed_interferences','hardware_interferences','band_interferences']),f
for f in ['Carriage strength probes.json','Carriage interface checks.json']:
 d=json.loads((O/f).read_text());assert all(a['passed'] for a in d),(f,[a for a in d if not a['passed']])
(O/'Joint checks.json').write_text(json.dumps([a for a in json.loads((O/'Carriage strength probes.json').read_text()) if 'pin' in a['region']],indent=2))
films=[]
for part in json.loads((R/'after thickness.json').read_text()):
 for sample in part['samples']:
  if sample['area']>1 and (sample['thickness']<.2 or (sample['thickness']<1 and max(abs(v) for v in sample['normal'])>.999999)):
   films.append(dict(part=part['part'],**sample))
(O/'Thin film check.json').write_text(json.dumps(films,indent=2))
assert not films,('Remaining thin planar films',films)
for s in [P/'package_preview.py',P/'render_review.py']:runpy.run_path(str(s),run_name='__main__')
shutil.copyfile(R/'Report.md',O/'Carriage strength report.md')
print('Carriage strength revision published.',flush=True)
