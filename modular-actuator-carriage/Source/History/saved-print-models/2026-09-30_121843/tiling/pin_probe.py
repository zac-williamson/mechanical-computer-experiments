import sys,numpy as np
from pathlib import Path
R=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(R/'latest-register-analysis/planar-register/register-from-multiplexer/Source'))
from ldraw_mesh import LDraw
l=LDraw('/Applications/Studio 2.0/ldraw/parts/4274.dat');a=l.mesh()[0].reshape(-1,3)*.4
print('bounds',a.min(0),a.max(0),flush=True)
for x in sorted(set(a[:,0].round(3))):
 b=a[np.abs(a[:,0]-x)<.0005];print(float(x),float(np.linalg.norm(b[:,1:],axis=1).max()),flush=True)
