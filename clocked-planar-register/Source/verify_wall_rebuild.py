"""Independent serial rebuild: compare solid geometry and operating metadata.

Exact tessellation is reported separately. Boolean triangulation and numeric
phase-cache roundoff may change vertex order; require symmetric solid volume
and bounds equivalence as well as identical functional part metadata.
"""
from pathlib import Path
import json,hashlib,tempfile,shutil,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Wall register';D=Path(tempfile.mkdtemp(prefix='wall-rebuild-'))
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
try:
 source=(R/'Source/wall_register.py').read_text().replace("OUT=ROOT/'Wall register'",'OUT=Path('+repr(str(D))+')')
 exec(compile(source,str(R/'Source/wall_register.py'),'exec'),{'__file__':str(R/'Source/wall_register.py'),'__name__':'__main__'})
 h=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest();b=hashlib.sha256((D/'geometry.npz').read_bytes()).hexdigest()
 ps=[json.load(open(d/'parts.json')) for d in [O,D]];vs=[np.load(d/'geometry.npz')['vertices'].reshape(-1,3) for d in [O,D]]
 maps=[{p['id']:p for p in pp} for pp in ps];rows=[]
 assert maps[0].keys()==maps[1].keys(),'Part inventory changed on rebuild'
 for n,p in maps[0].items():
  q=maps[1][n];a=vs[0][p['offset']//3:p['offset']//3+p['vertices']];bb=vs[1][q['offset']//3:q['offset']//3+q['vertices']]
  metadata_same={k:v for k,v in p.items() if k not in ['offset','vertices','bounds']}=={k:v for k,v in q.items() if k not in ['offset','vertices','bounds']}
  bounds_same=np.allclose(p['bounds'],q['bounds'],atol=1e-6,rtol=0);exact=a.shape==bb.shape and np.array_equal(a,bb);dv=0.
  if not exact:
   if p['kind']=='printed':
    sa,sb=solid(a),solid(bb);dv=(sa-sb).volume()+(sb-sa).volume()
   else:dv=0. if a.shape==bb.shape and np.allclose(a,bb,atol=1e-6,rtol=0) else float('inf')
  rows.append(dict(part=n,metadata_same=metadata_same,bounds_same=bool(bounds_same),exact_vertices=exact,symmetric_difference_mm3=dv,pass_check=bool(metadata_same and bounds_same and dv<.005)))
 result=dict(geometry_sha256=h,rebuilt_geometry_sha256=b,geometry_reproduction_pass=all(x['pass_check'] for x in rows),exact_mesh_reproduction_pass=h==b and ps[0]==ps[1],scope=__doc__,bounds_tolerance_mm=1e-6,symmetric_difference_tolerance_mm3=.005,parts=rows)
 (O/'Frame rebuild verification.json').write_text(json.dumps(result,indent=2));print('Rebuild differences',[x for x in rows if not x['exact_vertices'] or not x['pass_check']]);assert result['geometry_reproduction_pass']
finally:shutil.rmtree(D)
