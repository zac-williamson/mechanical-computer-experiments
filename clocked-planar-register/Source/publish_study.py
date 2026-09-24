"""Inspectable solid geometry; deliberately not a whole-machine animation."""
from pathlib import Path
import json
import numpy as np
import trimesh
from sequence import RAIL_LIMITS

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT.parent/'planar-register/register-from-multiplexer/Planar register'
parts=[]
for path in sorted((ROOT/'Cam study').glob('*.stl')):
    t=trimesh.load(path)
    name=path.stem
    if 'carriage' in name:
        t.apply_translation([3.749041,0,0])
    color=[.72,.37,.16] if 'rail' in name else [.81,.62,.12] if 'bolt' in name else [.47,.54,.46] if 'guide' in name else [.08,.52,.59]
    parts.append(dict(name=name,kind='rail' if 'rail' in name else 'bolt' if 'bolt' in name else 'fixed',
                      stage='master' if 'Master' in name else 'slave',color=color,
                      vertices=np.round(t.triangles.reshape(-1,3),5).flatten().tolist()))
hs=json.loads((BASE/'hardware.json').read_text())
v=np.load(BASE/'hardware.npz')['vertices']
for h in hs:
    if not h['id'].startswith('Cam roller'):
        continue
    a=v[h['offset']//3:h['offset']//3+h['vertices']]
    for stage,dx in [('master',0),('slave',160)]:
        parts.append(dict(name=stage+' '+h['id'],kind='bolt',stage=stage,color=[.25,.29,.32],
                          vertices=np.round(a+[dx,0,0],5).flatten().tolist()))
template=(ROOT/'Source/study.html').read_text()
(ROOT/'Cam study.html').write_text(template.replace('__DATA__',json.dumps(parts,separators=(',',':')))
                                  .replace('__MIN__',str(RAIL_LIMITS[0])).replace('__MAX__',str(RAIL_LIMITS[1])))
print('Published Cam study.html:',len(parts),'parts')
