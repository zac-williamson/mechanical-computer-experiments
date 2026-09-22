from pathlib import Path
import json,gzip,base64,numpy as np,trimesh
S=Path(__file__).resolve().parent;O=S.parent
D=json.loads((S/'scene.json').read_text());H=json.loads((O/'hardware.json').read_text());V=np.load(O/'hardware.npz')['vertices'];hardware={p['id']:V[p['offset']//3:p['offset']//3+p['vertices']] for p in H};arrays=[]
for p in D['parts']:
 n=p['id'];filename='Carriage fork and roof' if n=='Merged bearing and arm' else n
 if (O/(filename+'.stl')).exists():a=trimesh.load(O/(filename+'.stl')).triangles.reshape(-1,3)
 elif n in hardware:a=hardware[n]
 elif p['motion']=='band':a=np.zeros((3,3)) # replaced by the live band geometry on every draw
 else:raise ValueError(n)
 p.update(offset=sum(x.size for x in arrays),vertices=len(a));arrays.append(a)
D['trace']=json.loads((O/'Switching trace.json').read_text())['frames']
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode()
(O/'Viewer.html').write_text((S/'viewer.html.in').read_text().replace('__MODEL_DATA__',json.dumps(D,separators=(',',':'))))
