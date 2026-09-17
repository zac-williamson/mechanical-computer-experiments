from pathlib import Path
import subprocess,sys
for name in ['build.py','apply_phases.py','print_layout.py','publish.py']:
 subprocess.run([sys.executable,str(Path(__file__).parent/name)],check=True)
