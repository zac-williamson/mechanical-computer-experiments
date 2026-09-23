from pathlib import Path
import trimesh,manifold3d as m,numpy as np,json
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';out=O/'Quick lock fit test';out.mkdir(exist_ok=True)
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices),np.array(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
rows=[]
for name,source,clip,normal in [('Guide coupon','Unified rear backbone',box([-15.05,14,44],[4.95,38.4,70]),[0,1,0]),('Bolt','Direct lock bolt',None,[0,-1,0]),('Cam profile coupon','Write — Carriage fork and roof',box([-20,25.6,40],[-1,31,58]),[0,1,0])]:
 t=trimesh.load(O/(source+'.stl'))
 if clip is not None:
  a=(solid(t)^clip).simplify(.001).to_mesh64();t=trimesh.Trimesh(np.array(a.vert_properties)[:,:3],np.array(a.tri_verts),process=False);t.merge_vertices(digits_vertex=6);t.update_faces(t.nondegenerate_faces(height=1e-9));t.update_faces(t.unique_faces());t.remove_unreferenced_vertices()
 assert t.is_watertight and len(t.split())==1,(name,len(t.split()))
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(out/(name+'.stl'),file_type='stl_ascii');rows.append(dict(part=name,volume_cm3=t.volume/1000,dimensions_mm=t.extents.tolist()))
(out/'README.md').write_text('''# Quick lock fit test

These three pieces reproduce the actual guide clearances, bolt and cam profile. The shortened frame/cam are hand-held coupons, not substitute assembly parts and not an endurance or timing qualification.

Use the genuine LEGO 2L axle and two half bushes from the assembly. Print the supplied orientations in the intended PLA, layer height and wall settings. Remove any support only from non-sliding faces. Do not silently file the sliding faces: record any required adjustment and feed it back into the full model.

1. With no band, the bolt should move through its full 5.4 mm travel smoothly under light hand force, without sticking when the roller axle is pushed sideways.
2. Hold the cam behind the guide and move it through 7.5 mm. Check that the half bush rolls, the axle turns freely in its bore, and the bolt clears the lower guide without snagging.
3. Fit a light band to the actual rear anchors. Measure its force near both ends of travel; the calculation assumes 2 N, not an arbitrary tight band. Check reliable return from every intermediate position, including gentle sideways pressure on the roller.
4. Stop if the tip catches, the band rubs the frame, the roller skids heavily, or the guide visibly flexes. This test cannot validate the full frame, clutch timing, retained-bit capture or 0.1 Nm survival.

Do the full mechanism test only after this fit test. The coupon requires manually maintaining the cam's orientation; it is not a constrained motion jig.
''')
(O/'Fit coupon inventory.json').write_text(json.dumps(rows,indent=2));print(rows)
