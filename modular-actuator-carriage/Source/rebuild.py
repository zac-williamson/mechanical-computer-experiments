"""Rebuild latest rod revision, viewer, STLs and CAD checks from bundled inputs."""
from pathlib import Path
import runpy,json
R=Path(__file__).resolve().parent
for name in ['build.py','check.py','assembly.py','printcheck.py']:
 print('Running',name,flush=True)
 runpy.run_path(str(R/'rod-pin-spacing'/name),run_name='__main__')
c=json.loads((R/'build/Clearance checks.json').read_text())
assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
t=json.loads((R/'build/Tiling checks.json').read_text());assert t['passed'] and t['assembly_access']['passed']
a=json.loads((R/'build/Complete actuator assembly checks.json').read_text());assert a['passed']
print('REBUILD AND CAD CHECKS PASSED. Output:',R/'build',flush=True)
