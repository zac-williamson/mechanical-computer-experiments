"""Export and qualify every new modular part and the changed bit frames.

Normals check every downward face against 45 degrees. Independent 0.2 mm
layer slices check outward growth against the preceding slice offset by the
same height. Neither test treats disconnected-island checks as overhang proof.
"""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
O=Path(__file__).resolve().parents[1]/'Wall register';D=O/'Print oriented storage modules';D.mkdir(exist_ok=True)
P={p['id']:p for p in json.load(open(O/'parts.json'))};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def mesh(n):
 p=P[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
def solid(t):return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
from wall_print_geometry import qualify
entries=json.load(open(O/'Storage module schedule.json'))['print_orientations']+[dict(part='bit coordinated chassis '+str(i),axis=1,sign=-1) for i in [0,1]]
rows=[]
for e in entries:
 t,r=qualify(mesh(e['part']),e['axis'],e['sign']);path=D/(e['part']+'.stl');t.export(path);rows.append(dict(**e,**r,file=str(path.relative_to(O))));print(e['part'],r['print_geometry_pass'],r['unsupported_face_area_mm2'],r['maximum_unsupported_layer_growth_mm2'],flush=True)
# Compact production-module plate: real exported orientations, no auto-rotation.
layout=[];placements=[];x=0.;y=0.;row=0.
for e in rows:
 if not e['part'].startswith('master '):continue
 t=trimesh.load(O/e['file']);w,h=t.extents[:2]
 if x+w>240:x=0;y+=row+5;row=0
 t.apply_translation([x,y,0]);layout.append(t);placements.append(dict(part=e['part'],translation_mm=[x,y,0]));x+=w+5;row=max(row,h)
plate=trimesh.util.concatenate(layout);plate.export(D/'Master replaceable modules layout.stl')
(O/'Storage module print checks.json').write_text(json.dumps(dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,printer='Bambu Lab P2S, PLA Basic, 0.4 mm nozzle',parts=rows,layout=dict(size_mm=plate.extents.tolist(),placements=placements),modular_print_pass=all(r['print_geometry_pass'] for r in rows),slicer_validated=False,physically_validated=False),indent=2))
