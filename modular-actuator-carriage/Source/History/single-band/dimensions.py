from pathlib import Path
import os
R0=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R0/'neck-candidate');exec(compile((R0/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
for p in parts:
 if p['motion']=='bolt' or p['name']=='Locking bolt guide':print(p['name'],p['mesh'].bounds.tolist(),flush=True)
