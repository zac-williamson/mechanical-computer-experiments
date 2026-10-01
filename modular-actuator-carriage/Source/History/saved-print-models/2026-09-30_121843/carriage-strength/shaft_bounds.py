from pathlib import Path
import json,gzip,base64,numpy as np
r=Path(__file__).resolve().parents[1]/'adapted';d=json.loads((r/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3)
for p in d['parts']:
 if p['name'] not in ['reaction-stop-axle','pivot-stop-axle']:continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']];xy=(a.min(0)+a.max(0))[:2]/2
 for lo,hi in [(6,7.61),(7.61,24),(24,28.61),(28.61,30)]:
  b=a[(a[:,2]>=lo)&(a[:,2]<=hi)];print(p['name'],lo,hi,float(np.linalg.norm(b[:,:2]-xy,axis=1).max()) if len(b) else None,flush=True)
