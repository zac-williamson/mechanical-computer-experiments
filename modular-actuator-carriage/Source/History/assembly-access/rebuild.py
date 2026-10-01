from pathlib import Path
import runpy,sys
r=Path(__file__).resolve().parents[1]
runpy.run_path(str(r/'carriage-strength/run.py'),run_name='__main__')
sys.argv=['check.py','--verify'];runpy.run_path(str(r/'assembly-access/check.py'),run_name='__main__')
