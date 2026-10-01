from pathlib import Path
import runpy
R=Path(__file__).resolve().parent
runpy.run_path(str(R/'build_check.py'),run_name='__main__')
runpy.run_path(str(R/'package_candidate.py'),run_name='__main__')
