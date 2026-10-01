"""Package the checked intermediate revision without claiming print readiness."""
from pathlib import Path
import os,json,runpy,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'print-candidate'
os.environ['PLANAR_OUTPUT']=str(O)
for name in ['Clearance checks.json','Added carriage material checks.json']:
 d=json.loads((O/name).read_text())
 assert all(not d[k] for k in ['printed_interferences','hardware_interferences','band_interferences']),name
 assert all(p['watertight'] and p['solids']==1 for p in d['quality']),name
for name in ['Carriage strength probes.json','Carriage interface checks.json']:
 assert all(x['passed'] for x in json.loads((O/name).read_text())),name
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
shutil.copyfile(R/'design-review/Print geometry audit.json',O/'Print geometry audit.json')
runpy.run_path(str(R/'design-review/layer_check.py'),run_name='__main__')
shutil.copyfile(R/'design-review/Layer onset checks.json',O/'Layer onset checks.json')
guide_layers=[a for a in json.loads((O/'Layer onset checks.json').read_text()) if a['part']=='Locking bolt guide']
assert all(not a['unattached_islands'] for a in guide_layers),guide_layers
shutil.copyfile(R/'print-revision/Revision status.md',O/'README.md')
shutil.copyfile(R/'print-revision/Print notes.md',O/'Print notes.md')
shutil.copyfile(R/'carriage-strength/Report.md',O/'Carriage strength report.md')
runpy.run_path(str(R/'package_preview.py'),run_name='__main__')
runpy.run_path(str(R/'render_review.py'),run_name='__main__')
(O/'Checked mesh hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(O.glob('*.stl'))},indent=2))
print('Intermediate candidate packaged; carriage printability still unresolved.',flush=True)
