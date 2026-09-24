"""Find actual printed bearing rings around X-axis shafts, not just named walls.

This is radial support screening only. Axial retention, coupled shaft groups,
pins into ground and bearing load capacity remain separate requirements.
"""
from pathlib import Path
import hashlib,json
import numpy as np
import trimesh
R=Path(__file__).resolve().parents[1]/'Compact layout'
parts=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
prints=[]
for p in parts:
 if p['kind']!='printed':continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']]
 prints.append((p,trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)))
nums={'24316','3705','3706','3707','3708','3737','32073','32062','4519','60485','23948','44294','50450','50451'}
rows=[];angle=np.linspace(0,2*np.pi,16,endpoint=False)
for p in parts:
 if p.get('axis')!=0 or p.get('lego_part') not in nums:continue
 c=np.array(p['centre']);intervals=[]
 for pp,t in prints:
  lo=max(p['bounds'][0][0],t.bounds[0,0]);hi=min(p['bounds'][1][0],t.bounds[1,0])
  if hi-lo<.5 or np.any(c[1:]+3.5<t.bounds[0,1:]) or np.any(c[1:]-3.5>t.bounds[1,1:]):continue
  xs=np.arange(lo+.2,hi-.19,.25)
  if not len(xs):continue
  points=np.array([[[x,c[1]+r*np.cos(a),c[2]+r*np.sin(a)] for a in angle] for x in xs for r in [2.45,3.5]])
  hit=t.contains(points.reshape(-1,3)).reshape(len(xs),2,16)
  good=(hit[:,0].sum(1)==0)&(hit[:,1].sum(1)>=15)
  ids=np.flatnonzero(good)
  for group in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
   if not len(group):continue
   a,b=xs[group[0]]-.125,xs[group[-1]]+.125
   if b-a>=1.0:intervals.append(dict(part=pp['id'],x_mm=[float(a),float(b)],motion=pp.get('motion')))
 rows.append(dict(shaft=p['id'],bearings=intervals,has_any_radial_support=bool(intervals)))
report=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),shafts=rows,
 unsupported=[p['shaft'] for p in rows if not p['has_any_radial_support']],mechanically_qualified=False)
(R/'Shaft support screening.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
