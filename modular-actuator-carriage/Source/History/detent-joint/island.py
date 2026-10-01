exec(open('work/planar-module-restart/detent-joint/lock_check.py').read().split("bolt=solid")[0])
t=trimesh.load(O/'Carriage body.stl');t.apply_transform(trimesh.geometry.align_vectors([-1,0,0],[0,0,-1]));t.apply_translation(-t.bounds[0]);s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
for c in s.slice(5.7).decompose():
 if c.area()>.1 and (c^s.slice(5.5).offset(.21)).area()<.01:print('island',c.area(),c.bounds(),flush=True)
print('bounds original',trimesh.load(O/'Carriage body.stl').bounds.tolist(),flush=True)
