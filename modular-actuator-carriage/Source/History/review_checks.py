from pathlib import Path
import runpy
R=Path(__file__).resolve().parent
for name in ['validate.py','check_joints.py','package_preview.py','render_review.py']:
 print('Running',name,flush=True);runpy.run_path(str(R/name),run_name='__main__')
