"""Insertion depth into native LEGO axle joiners, including solid centre stops.

Axle/connector contacts are not blanket-exempted. This check uses the native
26287 centre web at +/-4 mm and 59443 divider at +/-0.1 mm. Cross-section fit,
retention, force capacity and other part intersections require separate checks.
"""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1]/'Compact layout'
ps=json.loads((R/'parts.json').read_text()); by={p['id']:p for p in ps}
shafts=[p for p in ps if p.get('axis')==0 and p.get('lego_part') in ['23948','44294','60485','4519','32062','32073','24316','3705','3706','3707','3708','3737','50450','50451']]
rows=[]
for p in ps:
 if not (p['id'].endswith(' L097') or p.get('lego_part')=='59443'):continue
 c=[(a+b)/2 for a,b in zip(*p['bounds'])]; half=4. if p['id'].endswith(' L097') else .1
 for a in shafts:
  if sum((x-y)**2 for x,y in zip(a['centre'][1:],c[1:]))>.0001:continue
  lo=max(a['bounds'][0][0],p['bounds'][0][0]);hi=min(a['bounds'][1][0],p['bounds'][1][0])
  if hi<=lo:continue
  overlap=max(0,min(hi,c[0]+half)-max(lo,c[0]-half))
  side='left' if a['centre'][0]<c[0] else 'right'
  clearance=c[0]-half-a['bounds'][1][0] if side=='left' else a['bounds'][0][0]-c[0]-half
  rows.append(dict(connector=p['id'],axle=a['id'],side=side,insertion_mm=hi-lo,centre_stop_penetration_mm=overlap,stop_clearance_mm=clearance))
result=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),interfaces=rows,centre_stop_pass=all(r['centre_stop_penetration_mm']<1e-6 for r in rows),positive_stop_clearance_pass=all(r['stop_clearance_mm']>=.19 for r in rows),mechanically_qualified=False)
(R/'Connector stop checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
