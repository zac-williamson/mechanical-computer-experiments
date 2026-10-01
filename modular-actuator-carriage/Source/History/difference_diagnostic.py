from pathlib import Path
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parent/'adapted'
for n in ['raw','repaired','removed','added']:
 a=np.load(R/('Carriage body '+n+'.npz'));s=m.Manifold(m.Mesh64(np.ascontiguousarray(a['vertices']),np.ascontiguousarray(a['faces'],dtype=np.uint64)))
 print(n,s.volume(),s.status(),flush=True)
 if n in ['removed','added']:
  for c in sorted(s.decompose(),key=lambda x:abs(x.volume()),reverse=True)[:8]:print(' piece',c.volume(),c.bounding_box(),flush=True)
def load(n):
 a=np.load(R/('Carriage body '+n+'.npz'));return m.Manifold(m.Mesh64(np.ascontiguousarray(a['vertices']),np.ascontiguousarray(a['faces'],dtype=np.uint64)))
a=load('raw');b=load('repaired');c=a.simplify(.001)
print('raw to simplified',abs((a-c).volume())+abs((c-a).volume()),flush=True)
print('simplified to repaired',abs((c-b).volume())+abs((b-c).volume()),flush=True)
assert abs((a-c).volume())+abs((c-a).volume())<.1
assert abs((c-b).volume())+abs((b-c).volume())<.1
import runpy
runpy.run_path(str(R.parent/'adapt.py'),run_name='__main__')
runpy.run_path(str(R.parent/'review_checks.py'),run_name='__main__')
