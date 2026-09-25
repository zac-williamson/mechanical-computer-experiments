"""Inventory actual shaft extents for a topology-first support redesign."""
from pathlib import Path
import csv,json,hashlib
import numpy as np
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
nums={'23948','44294','60485','4519','32062','32073','24316','3705','3706','3707','3708','3737','50450','50451'}
schedule=json.loads((O/'Bearing schedule.json').read_text())
assigned={n:b['shaft'] for b in schedule['bearings'] if b['status']=='complete bearing generated' for n in b['axle_parts']}
rows=[]
for p in P:
 if p.get('lego_part') not in nums:continue
 v=V[p['offset']//3:p['offset']//3+p['vertices']];lo=v.min(0);hi=v.max(0);axis=int(np.argmax(hi-lo));c=(lo+hi)/2
 rows.append(dict(part=p['id'],module=p['module'],axis='XYZ'[axis],length_mm=round(float(hi[axis]-lo[axis]),3),start_mm=round(float(lo[axis]),3),end_mm=round(float(hi[axis]),3),centre_x_mm=round(float(c[0]),3),centre_y_mm=round(float(c[1]),3),centre_z_mm=round(float(c[2]),3),support_status=('Scheduled transmission: '+assigned[p['id']]) if p['id'] in assigned else 'Retained actuator or moving-link pivot; physical fit unqualified'))
with (O/'Shaft layout inventory.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print(len(rows),'shaft pieces inventoried; geometry unchanged;',hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest())
