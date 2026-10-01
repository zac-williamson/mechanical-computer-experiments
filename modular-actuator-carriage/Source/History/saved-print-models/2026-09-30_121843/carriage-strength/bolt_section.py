from pathlib import Path
import trimesh,numpy as np
r=Path(__file__).resolve().parents[1]/'adapted'
t=trimesh.load(r/'Locking bolt.stl')
for z in [36.5,37,38,39,40.3]:
 s=t.section(plane_origin=[0,0,z],plane_normal=[0,0,1]);print(z,flush=True)
 if s:
  for p in s.discrete:print(np.round(p.min(0),3),np.round(p.max(0),3),flush=True)
