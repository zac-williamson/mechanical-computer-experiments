from pathlib import Path
import json,base64,gzip
import numpy as np
from pose import joint
R=Path(__file__).resolve().parents[1];O=R/'Assembly development'
parts=json.loads((O/'parts.json').read_text());v=np.load(O/'geometry.npz')['vertices']
cases=json.loads((R/'Angle-driven operation.json').read_text())['cases'];case=next(c for c in cases if c.get('frames'))
frames=case['frames']
for f in frames:f['joints']=[joint(p,f) for p in parts]
data=dict(parts=parts,frames=frames,geometry=base64.b64encode(gzip.compress(v.astype('<f4').tobytes())).decode(),
 ports=[['D',[-129,10.2,64]],['WRITE',[-157,10.2,114.4]],['CLK',[35,10.2,98.4]],['POWER',[-48,10.2,-16]],['Q',[260.2,10.2,0]]])
s=(R/'Source/assembly.html').read_text().replace('__DATA__',json.dumps(data,separators=(',',':')))
(R/'Viewer.html').write_text(s)
print('Published',len(parts),'parts and',len(frames),'angle-driven frames')
