"""All printed pairs at common angle-driven poses; no actuator blanket waiver."""
from pathlib import Path
import hashlib
import json,time,sys
import numpy as np,trimesh,manifold3d as m
from pose import transform
R=Path(__file__).resolve().parents[1];O=R/'Assembly development'
ps=[p for p in json.loads((O/'parts.json').read_text()) if p['kind']=='printed']
case=next(c for c in json.loads((R/'Angle-driven operation.json').read_text())['cases'] if c.get('frames'))
frames=case['frames'];indices=sorted(set(list(range(0,len(frames),10))+[len(frames)-1]))
if '--all-cases' in sys.argv:
 from operation import run
 cases=json.loads((R/'Angle-driven operation.json').read_text())['cases'];unique={}
 for case in cases:
  fs=run(tuple(case['start']),tuple(case['end']),case['initial_Q'],keep=True)['frames']
  for f in fs[::10]+fs[-1:]:
   signature=tuple(round(v,5) for bank in ['master','slave','write','clock'] for v in [f[bank]['q'],f[bank]['b']])+tuple(round(f[k],5) for k in ['rail','master_lift','slave_lift','master_ring','slave_ring'])
   if signature not in unique:unique[signature]=dict(f,case_start=case['start'],case_end=case['end'],initial_Q=case['initial_Q'])
 frames=list(unique.values());indices=list(range(len(frames)))
 print('Unique geometry samples',len(frames),flush=True)
solids={};boxes={}
for p in ps:
 t=trimesh.load(O/(p['id']+'.stl'))
 solids[p['id']]=m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
 assert solids[p['id']].status()==m.Error.NoError,p['id']
 boxes[p['id']]=trimesh.bounds.corners(t.bounds)
maxima={};tested=0;t0=time.monotonic()
for ii in indices:
 f=frames[ii];posed={};bounds={}
 for p in ps:
  T=transform(p,f);name=p['id'];posed[name]=solids[name].transform(T[:3,:]);corners=trimesh.transform_points(boxes[name],T);bounds[name]=[corners.min(0),corners.max(0)]
 for i,p in enumerate(ps):
  for q in ps[i+1:]:
   n1,n2=p['id'],q['id'];a,b=bounds[n1],bounds[n2]
   if np.any(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])<1e-5):continue
   tested+=1;vol=(posed[n1]^posed[n2]).volume()
   if vol>1e-4:
    key=(n1,n2)
    if key not in maxima or maxima[key]['volume_mm3']<vol:maxima[key]=dict(a=n1,b=n2,volume_mm3=vol,input_turns=f['turns'],frame=ii,case_start=f.get('case_start'),case_end=f.get('case_end'))
 if ii%100==0:print('Frame',ii,'pair tests',tested,'overlapping pairs',len(maxima),'seconds',round(time.monotonic()-t0,1),flush=True)
report=dict(scope='All printed pairs at sampled angle-driven poses; 112 transitions when --all-cases; not continuous clearance proof',all_cases='--all-cases' in sys.argv,samples=len(indices),pair_tests=tested,intersections=sorted(maxima.values(),key=lambda p:-p['volume_mm3']))
report['assembly_sha256']=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
(R/'Moving printed-part checks.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
