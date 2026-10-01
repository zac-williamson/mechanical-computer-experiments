from pathlib import Path
import os,runpy
r=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(r/'print-candidate')
for name in ['adapt.py','validate.py','carriage-strength/contact_check.py','design-review/print_audit.py','carriage-strength/probes.py','carriage-strength/functional_check.py']:
 print('STAGE',name,flush=True);runpy.run_path(str(r/name),run_name='__main__')

import sys
sys.argv=[str(r/"assembly-access/check.py"),"--verify"]
runpy.run_path(str(r/"assembly-access/check.py"),run_name="__main__")
