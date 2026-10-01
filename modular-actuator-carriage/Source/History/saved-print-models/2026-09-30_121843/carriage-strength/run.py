from pathlib import Path
import runpy,sys
r=Path(__file__).resolve().parents[1]
for script in [r/'adapt.py',r/'validate.py',r/'carriage-strength/contact_check.py']:
 runpy.run_path(str(script),run_name='__main__')
sys.argv=['audit.py','after'];runpy.run_path(str(r/'carriage-strength/audit.py'),run_name='__main__')

runpy.run_path(str(r/'carriage-strength/finalize.py'),run_name='__main__')
