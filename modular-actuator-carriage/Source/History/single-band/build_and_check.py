from pathlib import Path
import runpy
P=Path(__file__).resolve().parent
for n in ['build.py','check.py']:runpy.run_path(str(P/n),run_name='__main__')
