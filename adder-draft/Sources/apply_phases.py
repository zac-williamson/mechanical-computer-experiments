import os
from pathlib import Path
import json,math,numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT;p=OUT/'Assembly manifest.json';d=json.loads(p.read_text());g=json.loads((OUT/'Gear checks.json').read_text());ph=g['phases_degrees'];records={r['record_id']:r for r in d['records']}
# One phase per keyed routing shaft, separate from freely rotating clutch gears.
nets=[]
for prefix,count in [('P route gear ',7),('Cin route gear ',6)]:
 for i in range(count):
  n=prefix+str(i);v=np.array(records[n]['pos'])*.4;lo,hi=-1000,1000
  if n=='P route gear 0':hi=40
  if n=='P route gear 6':hi=40
  if n=='Cin route gear 3':lo=100
  if n=='Cin route gear 5':lo=58
  nets.append((v[1:],lo,hi,ph[n]))
for ac in d['actors']:
 n=ac+' R-P-idler-front';v=np.array(records[n]['pos'])*.4;nets.append((v[1:],-1000,1000,ph[n]))
for r in d['records']:
 n=r['record_id'];v=np.array(r['pos'])*.4;angle=ph.get(n)
 if angle is None and r['part']!='2780.dat':
  for yz,lo,hi,a in nets:
   if lo<v[0]<hi and np.linalg.norm(v[1:]-yz)<.001:angle=a;break
 if angle is None:continue
 prev=r.get('phase_deg',0);a=math.radians(angle-prev);c,s=math.cos(a),math.sin(a);R=np.array([[1,0,0],[0,c,-s],[0,s,c]]);r['matrix']=(R@np.array(r['matrix']).reshape(3,3)).reshape(-1).tolist();r['phase_deg']=angle
p.write_text(json.dumps(d,indent=2));print('Applied tooth phases and matching keyed shaft orientations')
