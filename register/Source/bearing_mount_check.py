from pathlib import Path
import json,numpy as np,trimesh
ROOT=Path(__file__).resolve().parent;import os;O=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));d=json.loads((O/'Assembly manifest.json').read_text());pins=[np.array(r['pos'])*.4 for r in d['records'] if r['part']=='2780.dat' and abs(r['pos'][1]*.4-28)<1e-5];rows=[]
for p in d['prints']:
 if 'bearing' not in p['id'].lower():continue
 t=trimesh.load(O/p['path']);lo,hi=t.bounds;found=[c.tolist() for c in pins if np.all(c>=lo-1e-4) and np.all(c<=hi+1e-4)]
 assert len(found)>=2,(p['id'],found)
 rows.append({'bearing':p['id'],'base_pin_centres_mm':found,'count':len(found)})
(O/'Bearing mount checks.json').write_text(json.dumps(rows,indent=2));print([(r['bearing'],r['count']) for r in rows])
