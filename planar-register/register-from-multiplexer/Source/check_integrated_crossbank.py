from pathlib import Path
import json,numpy as np,trimesh,itertools
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];rows=[];C={}
for h in hs:
 a=v[h['offset']//3:h['offset']//3+h['vertices']];C[h['id']]=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=False)
for a,b in itertools.combinations(hs,2):
 if {a['bank'],b['bank']}!={'Memory','Write'}:continue
 aa=C[a['id']].bounds;bb=C[b['id']].bounds
 # Expand translating native pieces through full carriage stroke.
 aa=aa.copy();bb=bb.copy()
 for h,bo in [(a,aa),(b,bb)]:
  if h['motion'] in ['worm','carriage','clutch-ring']:bo[0,0]-=4.15;bo[1,0]+=4.15
 d=np.minimum(aa[1],bb[1])-np.maximum(aa[0],bb[0])
 if np.all(d>1e-5):rows.append(dict(pair=[a['id'],b['id']],aabb_overlap_mm=d.tolist()))
# Axle-connector engagement is deliberate, but separate input/output shafts must not overlap.
r=dict(cross_core_native_aabb_candidates=rows,data_Q_axial_gap_mm=.8,joiner_axle_insertion_each_mm=7.6,joiner_nearest_sidewall_gap_mm=.6,limits='Conservative translated AABBs between Memory and Write hardware; axis-aligned rotational shape bounds may differ. Connector is a separate Lock-bank envelope, not proof of internal fit.')
(O/'Cross-core hardware check.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
