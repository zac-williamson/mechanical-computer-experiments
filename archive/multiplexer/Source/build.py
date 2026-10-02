"""Reproduce the accepted revision using the recovered original construction inputs."""
from pathlib import Path
import argparse,os,shutil,subprocess,sys
S=Path(__file__).resolve().parent;M=S.parent
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=M);args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,MUX_BUILD_OUTPUT=str(out))
for name in ['Switching trace.json','Change record.json']:
 if out!=M:shutil.copy2(M/name,out/name)
for script in ['generate_parts.py','build_viewer.py','print_layout.py']:
 subprocess.run([sys.executable,str(S/script)],env=env,check=True)
print('Build complete:',out)
