from pathlib import Path
import json,math,numpy as np
O=Path(__file__).resolve().parents[1];p=O/'Assembly manifest.json';d=json.loads(p.read_text());ph=json.loads((O/'Gear checks.json').read_text())['phases_degrees']
cx=d['actors']['C'][0];sx=d['actors']['S'][0];px=d['actors']['P'][0]
nets=[(10.2,16,-60,45,0),(10.2,32,-60,cx+40,ph['X L072']),
 (10.2,16,cx-40,cx+35,ph['C L072']),(10.2,16,sx-35,sx+40,ph['S L072']),
 (10.2,48,px-40,cx+40,ph['P L072']),(10.2,0,-100,400,0),
 (10.2+np.sqrt(80),24,-45,0,ph['X compound back']),
 (10.2+np.sqrt(80),40,px-45,px+8,ph['P compound back']),
 (10.2+np.sqrt(80),8,cx-45,cx,ph['C A compound back']),
 (10.2+np.sqrt(80),8,sx-8,sx+45,ph['S A compound back'])]
for r in d['records']:
 angle=ph.get(r['record_id'])
 if angle is None:
  x,y,z=np.array(r['pos'])*.4
  for yy,zz,lo,hi,value in nets:
   if abs(y-yy)<.001 and abs(z-zz)<.001 and lo<=x<=hi:angle=value;break
 if angle is None:continue
 a=math.radians(angle-r.get('phase_deg',0));c,s=math.cos(a),math.sin(a)
 R=np.array([[1,0,0],[0,c,-s],[0,s,c]])
 r['matrix']=(R@np.array(r['matrix']).reshape(3,3)).reshape(-1).tolist();r['phase_deg']=angle
p.write_text(json.dumps(d,indent=2))
