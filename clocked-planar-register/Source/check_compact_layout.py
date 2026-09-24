"""Limited static layout checks; explicitly not an operation/collision audit."""
from pathlib import Path
import json,itertools,hashlib
import numpy as np
R=Path(__file__).resolve().parents[1]/'Compact layout'
p=json.loads((R/'parts.json').read_text());by={x['id']:x for x in p}
axles=[x for x in p if x.get('lego_part') in ['32062','4519','3705','32073','3706','3707','3708','3737','50451','50450','60485','23948','44294'] and x.get('axis')==0]
hits=[]
for a,b in itertools.combinations(axles,2):
 if np.linalg.norm(np.array(a['centre'][1:])-b['centre'][1:])<.01:
  overlap=min(a['bounds'][1][0],b['bounds'][1][0])-max(a['bounds'][0][0],b['bounds'][0][0])
  if overlap>.01:hits.append([a['id'],b['id'],overlap])
port_parts={'D':'D external input 4L','WRITE':'WRITE input axle','CLK':'CLK incoming left 8L','POWER':'POWER incoming left 12L','Q':'slave output right 7L'}
ports={n:by[v]['bounds'][0 if n!='Q' else 1][0] for n,v in port_parts.items()}
assert all(abs(ports[n]+112)<.01 for n in ['D','WRITE','CLK','POWER'])
assert abs(ports['Q']-166.2)<.01
assert not hits,hits
pairs=[('CLK header gear 0','CLK header gear 2',24),('POWER header gear','master POWER 16T 32',16),('D header input 8T','D header output 8T',8)]
checks=[]
for a,b,pitch in pairs:
 d=float(np.linalg.norm(np.array(by[a]['centre'])[1:]-np.array(by[b]['centre'])[1:]));assert abs(d-pitch)<.001
 checks.append(dict(a=a,b=b,centre_distance_mm=d))
report=dict(scope='Static X-axis shaft intervals, three header gear centre distances, and port X positions only. No tooth phase, frame, swept collision or operation proof.',geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),axles=len(axles),overlaps=hits,ports_x_mm=ports,header_mesh_centres=checks)
(R/'Shaft checks.json').write_text(json.dumps(report,indent=2));print(f'PASS: {len(axles)} axle intervals, {len(checks)} header distances, five port positions; limited scope.')
