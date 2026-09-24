"""Conservative axle centreline envelope screening, separate from tooth contacts."""
from pathlib import Path
import json,numpy as np,trimesh
from pose import transform
R=Path(__file__).resolve().parents[1];O=R/'Assembly development';ps=json.loads((O/'parts.json').read_text())
axles=[]
for p in ps:
 name=p['id'].lower()
 if p['kind']!='native' or not ('axle' in name or 'shaft' in name or name=='continuous power axle'):continue
 if any(s in name for s in ['joiner','coupling','bush','retainer']):continue
 b=np.array(p['bounds']);dim=b[1]-b[0];axis=int(dim.argmax());c=b.mean(0);a=c.copy();z=c.copy();a[axis]=b[0,axis];z[axis]=b[1,axis];axles.append((p,np.array([a,z])))
frames=next(c['frames'] for c in json.loads((R/'Angle-driven operation.json').read_text())['cases'] if c.get('frames'));hits={}
for fi in [0,100,200,300,400]:
 lines=[(p,trimesh.transform_points(s,transform(p,frames[fi]))) for p,s in axles]
 for i,(p,a) in enumerate(lines):
  for q,b in lines[i+1:]:
   # All shaft axes in this design are X or Y; lever-mounted Y axes stay Y.
   ia=int(np.argmax(abs(a[1]-a[0])));ib=int(np.argmax(abs(b[1]-b[0])))
   aa=np.sort(a,axis=0);bb=np.sort(b,axis=0)
   if ia==ib:
    overlap=min(aa[1,ia],bb[1,ia])-max(aa[0,ia],bb[0,ia])
    if overlap<=.01:continue
    delta=a.mean(0)-b.mean(0);delta[ia]=0;dist=float(np.linalg.norm(delta))
   else:
    ca=a.mean(0);cb=b.mean(0);ca[ia]=np.clip(cb[ia],aa[0,ia],aa[1,ia]);cb[ib]=np.clip(ca[ib],bb[0,ib],bb[1,ib]);dist=float(np.linalg.norm(ca-cb))
   if dist<4.5:hits[(p['id'],q['id'])]=dict(a=p['id'],b=q['id'],centreline_distance_mm=dist,frame=fi)
report=dict(scope='Five poses, conservative 4.5 mm combined shaft envelope; not keyed shaft/gear or gear contact validation',axles=len(axles),candidates=list(hits.values()))
(R/'Axle path checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
