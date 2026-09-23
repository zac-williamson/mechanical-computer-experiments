from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Linkage development';P=O/'Development print orientations';P.mkdir(exist_ok=True)
# Choose the cardinal bed direction with greatest flat contact area. This is an
# orientation candidate, not a guarantee of support-free printing or good layer strength.
meta=json.loads((O/'printed-parts.json').read_text());out=[]
for p in meta:
 t=trimesh.load(O/(p['id']+'.stl'));choices=[]
 for axis in range(3):
  for sign in [-1,1]:
   n=np.eye(3)[axis]*sign;T=trimesh.geometry.align_vectors(n,[0,0,-1]);a=t.copy();a.apply_transform(T);a.apply_translation(-a.bounds[0]);bot=a.triangles[np.all(np.abs(a.triangles[:,:,2])<1e-4,axis=1)];area=float(np.linalg.norm(np.cross(bot[:,1]-bot[:,0],bot[:,2]-bot[:,0]),axis=1).sum()/2);choices.append((area,a,axis,sign))
 area,a,axis,sign=max(choices,key=lambda c:c[0]);a.export(P/(p['id']+'.stl'));out.append(dict(part=p['id'],bed_normal_original_axis='XYZ'[axis],bed_normal_sign=sign,bed_contact_mm2=area,print_size_mm=a.extents.tolist(),volume_mm3=float(t.volume),watertight=bool(t.is_watertight),solids=len(t.split())))
(P/'README.md').write_text('Development orientations only. These parts are NOT a validated print release. Each is placed on its largest cardinal flat face. Internal support access, bridging, pin-hole orientation, layer strength and band installation still need review in the slicer and physical prototypes. Use the separate Latch coupon for the first fit test. Do not print the complete register on the strength of a successful CAD sweep.\n')
(O/'Print orientation candidates.json').write_text(json.dumps(out,indent=2))
print('Wrote',len(out),'development orientation candidates; no unsupported-printing claim')
