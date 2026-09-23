from pathlib import Path
import shutil
root=Path(__file__).resolve().parent
out=root/'work/integrated-cam-development'
if out.exists():raise SystemExit('Working directory already exists; preserve its changes or remove it explicitly before restoring.')
shutil.copytree(root/'register-from-multiplexer/Planar register',out)
shutil.copy2(root/'frozen-inputs/Frame before detachable guide.stl',out)
print(out)
