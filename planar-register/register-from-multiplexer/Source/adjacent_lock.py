"""Adjacent spring-inserted lock. No blue carriage height extension."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
def so(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 z=s.to_mesh64();return trimesh.Trimesh(np.asarray(z.vert_properties)[:,:3],np.asarray(z.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=80)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 t=list(c);t[axis]=a;return s.translate(t)
meta=json.loads((O/'printed-parts.json').read_text());P={p['id']:so(trimesh.load(O/(p['id']+'.stl'))) for p in meta}
# Two pockets are beside the existing roof, at its existing height.
keeper=box([24.8,19.2,38.9],[46.3,29.6,45.3])
for x in [34.05,41.95]:keeper-=box([x-2.6,21.6,38.8],[x+2.6,27.2,45.4])
P['Memory — Carriage fork and roof']+=keeper
# Preserve the source bearing-flat minimum-X face. A separate planar link is pin-mounted.
w='Write — Carriage fork and roof'
P[w]+=box([82,15.6,54.0],[114,23.2,62.9])
# Flat two-pin mating land, clear of all original bearing surfaces.
P[w]-=box([81.1,23.2,53],[114.4,24.4,63.1])
link=box([58.8,23.4,53.3],[114,31,62.9])+box([44.95,23.4,48.7],[61.4,31,74.3])
link-=box([49.4,23.3,52.3],[57.0,31.1,70.3])
for x in [86,110]:
 P[w]-=cy(2.45,15.3,23.3,1,[x,0,58.1])+cy(3.2,22.4,23.3,1,[x,0,58.1])
 link-=cy(2.45,23.3,31.2,1,[x,0,58.1])+cy(3.2,23.3,24.2,1,[x,0,58.1])
# A shallow clearance on the outer face clears the existing pivot-axle tip.
# Print the opposite, uninterrupted Y=31 face on the bed.
for q in np.linspace(-4.6,4.6,47):link-=cy(4.0,23.3,25.2,1,[60-q,0,54])+cy(2.8,23.3,28.4,1,[60-q,0,54])
# A broad shear key carries side load; friction pins retain the joint.
P[w]+=box([90,23.1,54.5],[106,25.6,60.9])
link-=box([89.8,23.3,54.3],[106.2,25.8,61.1])
P['Write — Flat pin-mounted link']=link
meta.append(dict(id='Write — Flat pin-mounted link',bank='Write',source='new planar pin-mounted link',motion='carriage',print_rotation_axis=[1,0,0],print_rotation_angle=float(np.pi/2)))
# Closed bolt tip Z39.9; head lower face Z57. 0.4mm running gaps.
bolt=box([36.05,22,39.9],[39.95,26.8,57.1])+box([32,12,57],[50,27.8,61.8])
# Small chamfer on four tip edges for pocket entry.
cut=m.Manifold.cube([3.9,4.8,1]).translate([36.05,22,39.9]);bolt-=cut
poly=np.array([[36.45,22.4],[39.55,22.4],[39.55,26.4],[36.45,26.4]])
tip=m.CrossSection([poly]).extrude(1,scale_top=[3.9/3.1,4.8/4.0]) # centred below via own coordinates is unsuitable; use hull of boxes
bolt+=(box([36.45,22.4,39.9],[39.55,26.4,39.91])+box([36.05,22,40.89],[39.95,26.8,40.91])).hull()
# Crank pivot (60,*,54), input (53.2,*,64), output (42,*,62).
rock=(cy(4,4.4,12,1,[42,0,62])+cy(4,4.4,12,1,[53.2,0,64])+cy(5.6,4.4,12,1,[60,0,54])).hull()
for x,z in [(42,62),(60,54),(53.2,64)]:rock-=cy(2.6,4.3,12.1,1,[x,0,z])
# Remove the unused lower data/idler bearing tower on the left orange frame.
# Retain its upper clutch/WRITE bearings and two-pin rear mounting spine.
n='Write — Left side frame'
P[n]-=box([64,-5,-12],[74,26.8,9.8])
P[n]+=box([65,22.8,-6.4],[72.6,30.4,10.5])
P[n]-=cy(2.45,22.7,30.5,1,[68.8,0,-2.4])+cy(3.2,30.0,30.5,1,[68.8,0,-2.4])
# Open frame and vertical guide stay within the existing depth envelope.
base=P['Memory — Common baseboard']+P['Write — Common baseboard']+P['Write — Left side frame']+box([33.8,30.4,8],[66.8,38.4,12])
base+=box([32,30,33.3],[44,38.4,54.7])+box([32,19.2,45.7],[44,38.4,54.7])
base-=box([35.65,21.6,45.6],[40.35,27.2,54.8])
base+=box([67,0,37.5],[71,38.4,41.5])
for ya,yb in [(0,4),(16.8,20.8)]:
 base+=box([67,ya,40],[71,yb,62])+box([54.4,ya,46],[71,yb,50])+box([54.4,ya,48.4],[65.6,yb,59.6])
 base-=cy(2.6,ya-.1,yb+.1,1,[60,0,54])
# Integral rectangular lugs retain the band in the YZ plane at X24.6.
def lug(z):
 s=box([21.6,31,z],[27.2,37,z+4.8])
 s-=box([23.4,30.9,z-.1],[25.8,31.8,z+4.9])+box([23.4,36.2,z-.1],[25.8,37.1,z+4.9])
 return s
base+=lug(39.5)+box([26.8,31,39.5],[34,37,44.3]);bolt+=lug(57)+box([21.6,23,57],[34,27.8,61.8])+box([21.6,26,57],[23,32,61.8])
for xa,xb in [(32,34),(42,44)]:base+=box([xa,20.4,54.6],[xb,24.8,57])
# Clear the compact reversed-action follower; no separate WRITE inverter or bearings.
base-=box([54,23,52.9],[119,31.4,63.3])+box([39.95,23,48.3],[66.4,31.4,74.7])
# The rear projection of the head is unnecessary; keep the stem and front roller contact.
bolt-=box([40.1,23,56.9],[51,28,62])
# Clearance for the separate input roller through the moving head; output roller remains supported.
for q in np.linspace(-4.6,4.6,185):
 a=np.arctan2(6.8,10)+np.arcsin((q-6.8)/np.sqrt(146.24));up=max(0,54+18*np.sin(a)+8*np.cos(a)+3.6-57)
 bolt-=cy(4.0,11.9,28,1,[53.2+q,0,54+6.8*np.sin(a)+10*np.cos(a)-up])
 base-=cy(4.0,12,29.4,1,[53.2+q,0,54+6.8*np.sin(a)+10*np.cos(a)])
for n in ['Memory — Common baseboard','Write — Common baseboard','Write — Left side frame']:del P[n]
meta=[p for p in meta if p['id'] not in ['Memory — Common baseboard','Write — Common baseboard','Write — Left side frame']]
for n,s,mo in [('Adjacent lock frame',base,'fixed'),('Adjacent lock bolt',bolt,'bolt'),('Adjacent release crank',rock,'release-crank')]:P[n]=s;meta.append(dict(id=n,bank='Lock',source='new',motion=mo))
for p in meta:
 assert not P[p['id']].is_empty(), (p['id'],P[p['id']].status())
 t=mesh(P[p['id']]);t.export(O/(p['id']+'.stl'),file_type='stl_ascii');p.update(bounds=t.bounds.tolist(),watertight=bool(t.is_watertight),solids=len(t.split()))
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
hm=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];arr=[v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in hm]
def template(n):
 h=next(h for h in hm if h['id']==n);a=v[h['offset']//3:h['offset']//3+h['vertices']].copy();return a-(a.min(0)+a.max(0))/2
bush=template('Memory — A-input-bush-10')[:,[1,0,2]];shaft=template('Memory — A-idler-shaft')[:,[1,0,2]];stop=template('Memory — pivot-stop-axle')
def hw(n,a,c,mo,part,kind='native'):
 a=a+c;hm.append(dict(id=n,bank='Lock',source='new',motion=mo,lego_part=part,kind=kind,offset=sum(x.size for x in arr),vertices=len(a)));arr.append(a)
for x,z in [(42,62),(53.2,64)]:
 a=shaft.copy();a[:,1]*=(16 if x==42 else 32)/np.ptp(a[:,1]);hw('Lock roller axle '+str(x),a,[x,8.4 if x==42 else 12.4,z],'release-crank','32062' if x==42 else '3705','approximate axle ends')
 for y in ([2,14.4] if x==42 else [2,14.4,18.4,22.4,26.4]):hw(f'Lock half bush {x} {y}',bush,[x,y,z],'release-crank','32123a')
hw('Lock pivot 4L axle',shaft,[60,12,54],'fixed','3705');hw('Lock pivot retainer',bush,[60,-2,54],'fixed','32123a');hw('Lock rear pivot retainer',bush,[60,22.8,54],'fixed','32123a')
# Native LEGO friction pins, aligned with the two flat-link attachment bores.
pin=template('Memory — Baseboard pin X-28 Z-18.4')
for x in [86,110]:
 hw('Write link friction pin '+str(x),pin,[x,23.3,58.1],'carriage','2780')
 hm[-1]['bank']='Write'
(O/'hardware.json').write_text(json.dumps(hm,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr))
print('Non-single solids',[(p['id'],p['solids']) for p in meta if p['solids']!=1]);print('Blue carriage maximum Z',next(p['bounds'][1][2] for p in meta if p['id']=='Memory — Carriage fork and roof'))
