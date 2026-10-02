"""Fail closed when current compact geometry lacks required qualification evidence.

A successful script execution is not a mechanical pass. This command exits 2
unless every required report exists, matches the current geometry and passes.
Limited screens remain limited even when their individual predicates pass.
"""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1]/'Compact layout'
digest=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest()
requirements=[
 ('D header continuous swept separation','D route clearance checks.json','continuous_separation_pass'),
 ('Connector insertion and stop clearance','Connector stop checks.json','positive_stop_clearance_pass'),
 ('External gear profiles','External gear phase checks.json','external_profile_pass'),
 ('Recorded printed motion','Contact-solved printed motion.json','sampled_printed_motion_pass'),
 ('Native clutch surfaces at recorded poses','Native dog surface checks.json','sampled_dog_surface_pass'),
 ('Animated shaft direction consistency','Rotation path checks.json','rotation_paths_pass'),
 ('Nominal opposing shaft retention','Routing axial restraint.json','nominal_routing_restraint_pass'),
 ('Continuous all-part clearance, with tolerances','Continuous all-part qualification.json','qualified'),
 ('Loaded clutch dog engagement and release','Loaded clutch qualification.json','qualified'),
 ('All-axle 0.1 Nm load capacity','All axle load qualification.json','qualified'),
 ('Assembly and axial retention','Assembly retention qualification.json','qualified'),
 ('Printability, bearing finish and support-free base','Print orientation qualification.json','qualified'),
]
rows=[]
for label,filename,field in requirements:
 p=R/filename
 if not p.exists():status='MISSING'
 else:
  data=json.loads(p.read_text())
  status='STALE' if data.get('geometry_sha256')!=digest else ('PASS' if data.get(field) is True else 'NOT PASSED')
 rows.append(dict(check=label,report=filename,status=status))
ok=all(row['status']=='PASS' for row in rows)
result=dict(geometry_sha256=digest,checks=rows,mechanically_qualified=ok,print_release_allowed=ok)
(R/'Qualification summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
raise SystemExit(0 if ok else 2)
