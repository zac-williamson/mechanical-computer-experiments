"""Bounded local guide-placement probe, run serially under the project runner."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Functional register'
P=json.load(open(O/'parts.json'));L={p['id']:p for p in P};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
def solid(n):
 p=L[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
g=solid('slave timing guide 112.4');a=solid('slave cheek carrier');b=solid('slave bolt guide left')
for x in np.arange(100.6,111.2,.4):
 s=g.translate([x-112.4,0,0]);h1=s^a;h2=s^b
 print(round(x,2),round(h1.volume(),4),round(h2.volume(),4),list(h1.bounding_box()) if h1.volume()>.001 else '',flush=True)
