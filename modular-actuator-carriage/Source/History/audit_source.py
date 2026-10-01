from pathlib import Path
import re,json,gzip,base64
import numpy as np,trimesh,manifold3d as m
root=Path('/Users/zac/Documents/ChatGPT/lego designs 2_')
src=root/'register-from-multiplexer/Planar register'
d=json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',(src/'Viewer.html').read_text(),re.S)[1])
v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3)
rod=m.Manifold.cube([95.6,8,7.6]).translate([-47.8,6.2,28.2]);rows=[]
for p in d['parts']:
 if p.get('bank')!='Memory' or any(t in p['id'] for t in ['input-bush','idler','common 1','A-input','B-input','inner bearing']):continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 if p.get('kind') in ['printed','structure']:
  t=trimesh.load(src/(p['id']+'.stl'))
 else:t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
 s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 vol=(rod^s).volume() if s.status()==m.Error.NoError else None
 rows.append(dict(name=p['id'],bounds=t.bounds.tolist(),volume_in_carriage_rod_mm3=vol))
print(json.dumps(rows,indent=2),flush=True)
(root/'work/planar-module-restart/source-audit.json').write_text(json.dumps(rows,indent=2))
