from pathlib import Path
import runpy
r=Path(__file__).resolve().parent
for n in ['build.py','check.py','assembly.py','final_checks.py','finish.py']:
 print('START',n,flush=True);runpy.run_path(str(r/n),run_name='__main__');print('DONE',n,flush=True)
