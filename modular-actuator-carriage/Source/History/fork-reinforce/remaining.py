from pathlib import Path
import runpy
r=Path(__file__).resolve().parent
for name in ['finish.py','assembly.py','render.py']:runpy.run_path(str(r/name),run_name='__main__')
