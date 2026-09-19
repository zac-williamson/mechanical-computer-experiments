from pathlib import Path
import json, numpy as np, trimesh, manifold3d as m
ROOT=Path(__file__).resolve().parents[1]
PARAMS=json.loads((ROOT/'Design parameters.json').read_text())
pv=np.array(PARAMS['pivot'])
def solid(t):
    return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def load(name):return solid(trimesh.load(ROOT/(name+'.stl')))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,a,b):return m.Manifold.cylinder(b-a,r,circular_segments=128).translate([*pv,a])
def rotate(s,beta):return s.translate([-pv[0],-pv[1],0]).rotate([0,0,float(beta)]).translate([*pv,0])
def extr(poly,z,h):
    rings=[]
    for a in getattr(poly,'geoms',[poly]):
        if a.is_empty:continue
        rings.append(np.array(a.exterior.coords)[:-1]);rings.extend(np.array(i.coords)[:-1] for i in a.interiors)
    return m.CrossSection(rings,m.FillRule.EvenOdd).extrude(h).translate([0,0,z])
