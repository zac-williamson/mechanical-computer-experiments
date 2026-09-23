"""Installed cam/rocker development geometry. Reject on collisions; not load qualified."""
from pathlib import Path
import json,math,numpy as np,trimesh,manifold3d as m
from shapely.geometry import LineString,Polygon
R=Path(__file__).resolve().parents[1];A=R/'Assembly';O=R/'Linkage development';O.mkdir(exist_ok=True)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 z=s.to_mesh64();return trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=64)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 t=list(c);t[axis]=a;return s.translate(t)
def extrude_xz(poly,y0,y1):
 cs=m.CrossSection([np.asarray(poly.exterior.coords)[:-1]],m.FillRule.EvenOdd)
 for h in poly.interiors:cs-=m.CrossSection([np.asarray(h.coords)[:-1]])
 return cs.extrude(y1-y0).rotate([90,0,0]).translate([0,y1,0])
meta=json.loads((A/'printed-parts.json').read_text());P={p['id']:solid(trimesh.load(A/(p['id']+'.stl'))) for p in meta}
# Lengthen the bolt below its shaft: retain clearance above its lowered head.
nb='Memory — Lock bolt';old=P[nb];head=old^box([-12,40,-1],[12,49,8.91]);shaft=old-box([-12,40,-1],[12,49,8.91]);P[nb]=shaft+head.translate([0,0,-4])+box([-1.95,43.6,4.8],[1.95,48.4,8.95])
# Longer sleeve maintains guidance throughout tolerance-expanded opening travel.
n='Memory — Fixed lower latch guide'
P[n]+=box([-6,40,7.3],[6,52,20.5])-box([-2.35,43.2,7.2],[2.35,48.8,20.6])
# Stop faces and connection lie ahead of the output roller axle.
for x0,x1 in [(-12,-8),(8,12)]:P[n]+=box([x0,49.6,4.9],[x1,58.8,20.5])
P[n]+=box([-12,49.6,17.3],[12,52.4,20.5])
P[n]-=box([-5.6,52.3,16.1],[5.6,56.4,18])
# Bolt crosshead, attached by two LEGO friction pins (no printed pins).
cap=box([-11.2,48.8,-3.1],[11.2,58.8,4.9])
for x in [-6.4,6.4]:
 for name in ['cap','bolt']:
  s=cap if name=='cap' else P['Memory — Lock bolt']
  s-=cyl(2.45,40.5,56.7,1,[x,0,.9])+cyl(3.2,48.1,49.3,1,[x,0,.9])
  if name=='cap':cap=s
  else:P['Memory — Lock bolt']=s
# Broad integral rectangular band lugs, not pivots or structural joining pins.
cap+=box([-19,55.2,-3.1],[-9,58.4,2.9])+box([-19,55.2,-15.1],[-15,63.2,2.9])+box([-19,58.8,-15.1],[-9,63.2,-11.1])
cap-=box([-13,59.6,-15.2],[-10.6,62,-13.1])
# Rocker represented at zero angle; all pivot axes Y, all working tooth geometry unchanged.
rock=extrude_xz(Polygon([(0,2.7),(12,-1.8),(50,-1.8),(50,15.2),(12,15.2),(0,10.7)]),58.8,66.4)+cyl(8.5,58.8,66.4,1,[50,0,6.7])+cyl(5,58.8,66.4,1,[0,0,6.7])
rock-=box([-6,62.8,0],[6,66.5,14])
rock+=box([-14,58.8,.7],[0,62.8,4.7])
rock-=box([-14.1,59.6,2.7],[-11,62,4.8])
for x in [0,40,50]:rock-=cyl(2.6,58.7,66.5,1,[x,0,6.7])
# Cam slot is the swept circle of a native half-bush roller, not a point-follower curve.
def command(q):return -15+20*float(np.clip((q-.9)/1.5,0,1))
def centre(q):
 u=(5.8+command(q))/50;return (50-10*math.sqrt(1-u*u)+q,6.7+10*u)
slot=LineString([centre(q) for q in np.linspace(-8,8,3201)]).buffer(3.8,quad_segs=48)
cam=box([28,67.2,-8],[98,70.4,24])-extrude_xz(slot,67.1,70.5)
cam+=box([81.2,67.2,-12],[90.4,74.8,8])
# Right moving support carries the cam; its original clutch bearing remains intact.
w='Write — Right carriage bearing support'
P[w]+=box([81.2,27.4,-11.5],[88.8,75.6,-.5])+box([81.2,68,-16],[90.4,75.6,4])
for z in [-8,4]:
 P[w]-=cyl(2.45,67.9,75.7,1,[85,0,z-4])+cyl(3.2,75.1,75.7,1,[85,0,z-4])
 cam-=cyl(2.45,67.1,75,1,[85,0,z])+cyl(3.2,67.1,67.7,1,[85,0,z])
b='Common two-core baseboard';P[b]-=box([76.2,28.4,-11.9],[95.4,38.5,-.1])
cam=cam.translate([0,8.8,-4]);rock=rock.translate([0,0,-4])
# Open rear frame: lower cam rail, rear spine, pivot cheeks; upper rail is removable.
frame=box([15,63.2,-16.4],[111,74.8,-12.4])
for ya,yb in [(63.2,66.8),(70.8,74.8)]:frame+=box([15,ya,-12.4],[111,yb,-4])
for xa,xb in [(15,23),(103,111)]:frame+=box([xa,66.8,-12.4],[xb,74.8,24])
frame+=box([44.4,71.2,-12.4],[60,78.8,24])
frame=frame.translate([0,8.8,0])
frame+=box([38,38.8,-12.4],[62,83.6,-8.4])+box([38,38.8,-8.4],[62,46.4,6])
for xa,xb in [(44.4,47.6),(53.6,57.6)]:frame+=box([xa,46.4,-8.4],[xb,70.8,-4.4])
for ya,yb in [(54.4,58.4),(66.8,70.8)]:frame+=box([44.4,ya,-8.4],[57.6,yb,12.3])
frame-=cyl(2.6,54.3,70.9,1,[50,0,6.7])+cyl(3.6,70.8,79,1,[50,0,6.7])
upper=box([15,63.2,24.4],[111,74.8,32])
for ya,yb in [(63.2,66.8),(70.8,74.8)]:upper+=box([23.4,ya,20],[102.6,yb,24.4])
upper-=box([42.8,70.7,19.9],[60.4,74.9,24.39])
upper+=box([43.2,70.8,24.4],[60,78.8,32])
upper=upper.translate([0,8.8,0])
cap_pins=[(19,79.6),(107,79.6),(47.2,83.6),(55.2,83.6)]
for x,y in cap_pins:
 frame-=cyl(2.45,16.1,24.1,2,[x,y,0])+cyl(3.2,23.5,24.1,2,[x,y,0])
 upper-=cyl(2.45,24.3,32.3,2,[x,y,0])+cyl(3.2,24.3,24.9,2,[x,y,0])
for x in [44,56]:
 frame-=cyl(2.45,38.7,46.7,1,[x,0,2])+cyl(3.2,38.7,39.3,1,[x,0,2])
 P[b]-=cyl(2.45,30.3,38.5,1,[x,0,2])+cyl(3.2,37.9,38.5,1,[x,0,2])
frame=frame.translate([0,0,-4]);upper=upper.translate([0,0,-4])
# Preserve ample ligament below relocated baseboard mount holes.
P[b]+=box([33.8,30.4,-8],[66.8,38.4,-3.9])
for x in [44,56]:P[b]-=cyl(2.45,30.3,38.5,1,[x,0,-2])+cyl(3.2,37.9,38.5,1,[x,0,-2])
# Relieve the cam arm's lower sweep from the front rail only.
frame-=box([76.2,71.9,-16.1],[95.4,83.7,0])
new=[('Lock crosshead',cap,'bolt'),('Lock rocker',rock,'rocker'),('Write cam',cam,'write-cam'),('Rear linkage frame',frame,'fixed'),('Upper cam rail',upper,'fixed')]
for n,s,mo in new:P[n]=s;meta.append(dict(id=n,bank='Linkage',motion=mo,source='new'))
for p in meta:
 t=mesh(P[p['id']]);print(p['id'],P[p['id']].status()) if t.bounds is None else None;t.export(O/(p['id']+'.stl'));p.update(bounds=t.bounds.tolist(),watertight=bool(t.is_watertight),solids=len(t.split()))
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
# Native LEGO pins and bushes; standard shorter axles use dimension-adjusted source
# cross sections and are labelled approximate, never presented as native CAD.
hm=json.loads((A/'hardware.json').read_text());v=np.load(A/'hardware.npz')['vertices'];arr=[v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in hm]
def template(key):
 h=next(p for p in hm if p['id']==key);a=v[h['offset']//3:h['offset']//3+h['vertices']].copy();return a-(a.min(0)+a.max(0))/2
pin=template(next(p['id'] for p in hm if p['id'].startswith('Memory — Baseboard pin')))
bush=template('Memory — A-input-bush-10');bush=bush[:,[1,0,2]]
shaft=template('Memory — A-idler-shaft');shaft=shaft[:,[1,0,2]]
def hw(n,a,c,part,mo='fixed',kind='native'):
 a=a+np.array(c);hm.append(dict(id=n,bank='Linkage',source='new',lego_part=part,motion=mo,kind=kind,offset=sum(x.size for x in arr),vertices=len(a)));arr.append(a)
for x in [-6.4,6.4]:hw('Crosshead pin '+str(x),pin,[x,48.6,.9],'2780','bolt')
for z in [-8,4]:hw('Cam attachment pin '+str(z),pin,[85,75.8,z-4],'2780','write-cam')
for x in [44,56]:hw('Linkage mount pin '+str(x),pin,[x,38.6,-2],'2780')
for x,y in cap_pins:hw('Cam rail pin '+str(x),pin[:,[0,2,1]],[x,y,20.2],'2780')
for x,ys in [(0,[56.4,65.2]),(40,[56.4,68.8,72.8,77.6]),(50,[52.4])]:
 for y in ys:hw(f'Rocker half bush {x} {y}',bush,[x,y,2.7],'32123a','rocker' if x!=50 else 'fixed')
for x,cy,length,part in [(0,60.8,16,'32062'),(40,68,32,'3705')]:
 a=shaft.copy();a[:,1]*=length/np.ptp(a[:,1]);hw('Rocker axle '+str(x),a,[x,cy,2.7],part,'rocker','approximate axle end geometry')
a=template('Memory — pivot-stop-axle');hw('Rocker pivot axle',a,[50,59.7,2.7],'24316')
(O/'hardware.json').write_text(json.dumps(hm,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr))
print('Non-single solids:',[(p['id'],p['solids']) for p in meta if p['solids']!=1]);print('Wrote development linkage')
