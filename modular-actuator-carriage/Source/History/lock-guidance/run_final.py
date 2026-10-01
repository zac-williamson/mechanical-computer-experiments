import runpy
from pathlib import Path
r=Path(__file__).resolve().parent
for name in ['build.py','check.py','entry.py','bands.py','finish.py']:
 print('START',name,flush=True);runpy.run_path(str(r/name),run_name='__main__');print('DONE',name,flush=True)
