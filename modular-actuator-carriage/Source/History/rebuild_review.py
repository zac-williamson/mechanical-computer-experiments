from pathlib import Path
import runpy
r=Path(__file__).resolve().parent
for name in ['adapt.py','review_checks.py']:runpy.run_path(str(r/name),run_name='__main__')
