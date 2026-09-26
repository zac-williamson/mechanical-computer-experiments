"""Rebuild sequentially in scratch and compare exact geometry and part metadata."""
from pathlib import Path
import json,hashlib,tempfile,shutil
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
D=Path(tempfile.mkdtemp(prefix='wall-rebuild-'))
try:
 source=(R/'Source/wall_register.py').read_text().replace("OUT=ROOT/'Wall register'",'OUT=Path('+repr(str(D))+')')
 exec(compile(source,str(R/'Source/wall_register.py'),'exec'),{'__file__':str(R/'Source/wall_register.py'),'__name__':'__main__'})
 h=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest();b=hashlib.sha256((D/'geometry.npz').read_bytes()).hexdigest()
 same=json.loads((O/'parts.json').read_text())==json.loads((D/'parts.json').read_text())
 result=dict(geometry_sha256=h,rebuilt_geometry_sha256=b,geometry_reproduction_pass=h==b and same,exact_mesh_reproduction_pass=h==b,scope='Independent full source rebuild in an empty temporary directory; exact geometry bytes and all part/pose metadata compared.')
 (O/'Frame rebuild verification.json').write_text(json.dumps(result,indent=2));assert result['geometry_reproduction_pass'],result
finally:shutil.rmtree(D)
