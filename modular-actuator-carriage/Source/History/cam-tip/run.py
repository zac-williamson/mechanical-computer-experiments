from pathlib import Path
import runpy
r=Path(__file__).resolve().parent
for n in ['build.py','printcheck.py']:runpy.run_path(str(r/n),run_name='__main__')
