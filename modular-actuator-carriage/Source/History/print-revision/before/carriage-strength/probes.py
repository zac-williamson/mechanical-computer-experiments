from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parent;O=R.parent/'adapted'
def load(n):
 t=trimesh.load(O/(n+'.stl'));return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=48)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 c=list(c);c[axis]=a;return s.translate(c)
parts={n:load(n) for n in ['Carriage body','Carriage bearing end']};out=[]
def probe(n,label,s):
 vol=(s-parts[n]).volume();out.append(dict(part=n,region=label,missing_material_mm3=vol,passed=vol<.02))
probe('Carriage body','Front wall: 3.15 mm along Y',box([-7,1.5501,14],[7,4.6999,24]))
for i,x in enumerate([-8.8,-1.3]):
 probe('Carriage body',f'Pocket {i+1} front wall: 3 mm Y',box([x-2.3,20.5001,36],[x+2.3,23.4999,39.3999]))
 probe('Carriage body',f'Pocket {i+1} rear wall: 3 mm Y',box([x-2.3,28.5001,36],[x+2.3,31.4999,39.3999]))
probe('Carriage body','Pocket divider: 2.8 mm X',box([-6.4499,23.5001,35.8001],[-3.6501,28.4999,39.3999]))
probe('Carriage body','Pocket end wall: 3.05 mm X',box([1.0501,23.5001,35.8001],[4.0999,28.4999,39.3999]))
probe('Carriage body','Central clutch bridge: 2.9 mm clear rectangular core',box([-7,9.8,7.7],[7,10.6,10.6]))
probe('Carriage body','Lever back wall: 3 mm Y',box([-7,37.5001,12],[25,40.4999,19.8]))
for n,a,b,x in [('Carriage body',-16.24,-9.11,-11),('Carriage bearing end',9.11,16.24,11)]:
 probe(n,'Bearing bridge: 3.98 mm rectangular core',box([a,10.1,9.16],[b,10.3,13.14]))
 probe(n,'Actuator bearing: 3.05 mm radial core',cy(5.95,a,b,0,[0,10.2,16])-cy(2.9,a-.01,b+.01,0,[0,10.2,16]))
 for y,z,label in [(24.7,.5,'front'),(35.4,32,'rear')]:
  aa,bb=(7.81,7.99) if n=='Carriage body' else (8.01,8.59)
  # Main body collar ends at its split face X7.8; check just inside that face.
  if n=='Carriage body':aa,bb=7.4,7.79
  probe(n,label+' joining pin: 2 mm collar surround',cy(5.29,aa,bb,0,[0,y,z])-cy(3.31,aa-.01,bb+.01,0,[0,y,z]))
 probe(n,'Rod pin: 2 mm surround over full engaged length',cy(4.49,14.81,22.3,1,[x,0,32])-cy(2.51,14.8,22.31,1,[x,0,32]))
 probe(n,'Rod pin collar: 2 mm surround',cy(5.24,14.41,14.79,1,[x,0,32])-cy(3.26,14.4,14.8,1,[x,0,32]))
for n,a,b in [('Carriage body',-16.2,-13.5),('Carriage bearing end',13.5,16.2)]:
 probe(n,'Bearing-to-rod-boss web: 2.7 x 2.8 mm continuous core',box([a,14,19],[b,16.8,27.6]))
(O/'Carriage strength probes.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2),flush=True)
