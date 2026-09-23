from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register';B=R.parent/'work/register-mux-reference/multiplexer';trace=json.loads((B/'Switching trace.json').read_text())['frames']
def so(t):return m.Manifold(m.Mesh64(np.array(t.vertices,order='C'),np.array(t.faces,dtype=np.uint64,order='C')))
def tm(s):
 d=s.to_mesh64();return trimesh.Trimesh(np.asarray(d.vert_properties)[:,:3],np.asarray(d.tri_verts),process=False)
results=[]
for bank in ['Memory','Write']:
 old=trimesh.load(B/'Carriage fork and roof.stl');lever=trimesh.load(B/'Short lever.stl');pivot=np.array([13.192323604,10.2,32.128448698]);shift=np.array([0,0,0]) if bank=='Memory' else np.array([96.8,0,16]);old.apply_translation(shift);lever.apply_translation(shift);pivot+=shift
 new=trimesh.load(O/(bank+' — Carriage fork and roof.stl'));delta=so(new)-so(old)
 # Keep the deliberate addition region; boolean round-trip slivers on inherited STL surfaces are not new features.
 cut=m.Manifold.cube([250,30,140]).translate([-60,12.1 if bank=='Memory' else 18.9,-30]);added=tm(delta^cut);ca=trimesh.collision.CollisionManager();cl=trimesh.collision.CollisionManager();ca.add_object('addition',added);cl.add_object('lever',lever);hits=[]
 for i,f in enumerate(trace):
  T=np.eye(4);T[0,3]=f['q'];ca.set_transform('addition',T);cl.set_transform('lever',trimesh.transformations.rotation_matrix(np.radians(f['b']),[0,1,0],pivot))
  if ca.in_collision_other(cl):hits.append(i)
 results.append(dict(bank=bank,source_trace_frames=len(trace),new_carriage_material_to_lever_contacts=hits))
rep=dict(results=results,qualification='Checks the additions against the original complete lever/contact trace. Inherited worm/reaction/lever tooth geometry is unchanged; original tooth-contact study remains applicable. Not force or endurance validation.')
(O/'Actuator addition checks.json').write_text(json.dumps(rep,indent=2));print(json.dumps(rep,indent=2))
