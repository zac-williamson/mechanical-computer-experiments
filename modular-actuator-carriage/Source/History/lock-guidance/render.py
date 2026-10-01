from pathlib import Path
import os,runpy
r=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(r/'lock-guidance-candidate');runpy.run_path(str(r/'render_review.py'),run_name='__main__')
