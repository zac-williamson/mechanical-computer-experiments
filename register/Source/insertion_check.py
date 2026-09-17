from pathlib import Path
import numpy as np,trimesh,manifold3d as m,json
ROOT=Path(__file__).resolve().parent;import os; O=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)))
def load(n):
 t=trimesh.load(O/(n+'.stl'));return m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
h=load('K Front actuator bridge');b=load('HOLD side bolt');hits=[]
# Housing off bridge; follower pin and elastic band installed afterward.
for z in np.arange(-2.8,45.01,.1):
 v=(h^b.translate([0,0,float(z)])).volume()
 if v>1e-6:hits.append([float(z),v])
assert not hits,hits[:10]
r={'assembly':'Slide complete printed bolt from +Z into front bridge before installing rear bridge; fit follower through upper slot afterward, then mount bridges, fit cam and band.','step_mm':.1,'translation_range_mm':[-2.8,45],'intersections':hits}
(O/'Insertion checks.json').write_text(json.dumps(r,indent=2));print(r)
