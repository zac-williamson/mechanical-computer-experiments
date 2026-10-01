from pathlib import Path
import runpy
R=Path(__file__).resolve().parent
for n in ['extra.py','finish.py','render.py']:
 print('RUN',n,flush=True);runpy.run_path(str(R/n),run_name='__main__')
