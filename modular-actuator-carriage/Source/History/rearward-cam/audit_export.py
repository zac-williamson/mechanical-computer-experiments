from pathlib import Path
import trimesh
O=Path(__file__).resolve().parents[1]/'rearward-cam-candidate'
for tol in [1e-8,1e-9,1e-10]:
 trimesh.constants.tol.merge=tol
 t=trimesh.load_mesh(O/'Print layout.stl');print(tol,t.is_watertight,len(t.split(only_watertight=False)),[(c.bounds.tolist(),c.volume,c.is_watertight) for c in t.split(only_watertight=False)],flush=True)
