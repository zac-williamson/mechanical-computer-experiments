from geometry import *
from trimesh.collision import CollisionManager
O=OUT;h=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];p=next(p for p in h if p['id']=='L099');vv=v[p['offset']//3:p['offset']//3+p['vertices']];ring=trimesh.Trimesh(vv,np.arange(len(vv)).reshape(-1,3),process=True)
fork=solid(trimesh.load(Path(__file__).parent/'Clutch interface reference.stl'));car=solid(trimesh.load(O/'Carriage fork and roof.stl'))+solid(trimesh.load(O/'Right carriage bearing support.stl'));added=car-fork
cm=CollisionManager();cm.add_object('added',mesh(added));hits=[]
for q in np.linspace(-4.6,4.575,39):
 cm.set_transform('added',trimesh.transformations.translation_matrix([q,0,0]))
 for a in np.linspace(0,360,73):
  T=trimesh.transformations.rotation_matrix(np.radians(a),[1,0,0],[0,10.2,0]);T[0,3]=np.sign(q)*max(abs(q)-.4,0);yes,data=cm.in_collision_single(ring,T,return_data=True)
  if yes:hits.append(dict(q=float(q),angle=float(a),points=[x.point.tolist() for x in data[:3]]));break
r=dict(new_material_ring_interferences=hits,ring_watertight=bool(ring.is_watertight),retained_fork='Original fork geometry below Z7.5 excluded from the comparison.');(O/'fork-check.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
