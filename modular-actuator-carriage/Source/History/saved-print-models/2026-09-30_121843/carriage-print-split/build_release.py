"""Build and check the revised carriage in staging; publishing is a separate step."""
from pathlib import Path
import runpy,os,json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'split-candidate';os.environ['PLANAR_OUTPUT']=str(O)
for name in ['adapt.py','validate.py','carriage-strength/contact_check.py','carriage-print-split/audit_carriage.py','carriage-print-split/check_assembly.py','carriage-print-split/layer_screen.py','design-review/print_audit.py']:
 print('STAGE',name,flush=True);runpy.run_path(str(R/name),run_name='__main__')
for name in ['Clearance checks.json','Added carriage material checks.json']:
 d=json.loads((O/name).read_text());assert all(p['watertight'] and p['solids']==1 for p in d['quality']),name
 assert all(not d[k] for k in ['printed_interferences','hardware_interferences','band_interferences']),name
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
for name in ['Carriage interface checks.json','Carriage strength probes.json']:assert all(c['passed'] for c in json.loads((O/name).read_text())),name
shutil.copy2(R/'design-review/Print geometry audit.json',O/'Print geometry audit.json')
shutil.copy2(R/'carriage-print-split/README.md',O/'README.md')
shutil.copy2(R/'carriage-print-split/README.md',O/'Carriage strength report.md')
for name in ['package_preview.py','render_review.py']:runpy.run_path(str(R/name),run_name='__main__')
(O/'Checked mesh hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.stl')},indent=2))
print('Revised two-piece carriage checked and packaged in staging. Bearing coupon still requires a physical print test.',flush=True)
