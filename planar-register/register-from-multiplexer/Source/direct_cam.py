"""Direct roller-cam lock, supported by the existing memory bearing cheek.
Builds from the frozen, mux-derived pre-cam assembly, never an unrelated register.
"""
from pathlib import Path
import json,shutil,numpy as np,trimesh,manifold3d as m
from direct_cam_math import RADIUS,SLOPE,CLOSED_Z,HIGH,INTERCEPT,CREST
R=Path(__file__).resolve().parents[1];O=R/'Planar register';B=R.parent/'work/register-before-direct-cam';SRC=R.parent/'work/register-mux-reference/multiplexer'
if not B.exists():
 B.mkdir()
 for p in O.glob('*.stl'):shutil.copy2(p,B/p.name)
 for n in ['printed-parts.json','hardware.json','hardware.npz']:shutil.copy2(O/n,B/n)
def solid(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(np.array(a.vert_properties)[:,:3],np.array(a.tri_verts),process=False)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cy(r,a,b,axis,c):
 s=m.Manifold.cylinder(b-a,r,circular_segments=96)
 if axis==0:s=s.rotate([0,90,0])
 if axis==1:s=s.rotate([-90,0,0])
 v=list(c);v[axis]=a;return s.translate(v)
def xz(poly,y0,y1):return m.CrossSection([np.array(poly)],m.FillRule.EvenOdd).extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
meta=json.loads((B/'printed-parts.json').read_text());P={p['id']:solid(trimesh.load(B/(p['id']+'.stl'))) for p in meta}
for n in ['Adjacent lock frame','Adjacent lock bolt','Adjacent release crank','Write — Flat pin-mounted link']:P.pop(n)
meta=[p for p in meta if p['id'] in P]
# Preserve the original two-piece bearing carriages and their bearing-flat bed faces.
P['Memory — Carriage fork and roof']=solid(trimesh.load(SRC/'Carriage fork and roof.stl'))
keeper=box([24.8,14.8,38.9],[46.3,24.8,45.3])
for x in [34.05,41.95]:keeper-=box([x-2.6,16.8,38.8],[x+2.6,22.4,45.4])
P['Memory — Carriage fork and roof']+=keeper
w='Write — Carriage fork and roof';P[w]=solid(trimesh.load(SRC/'Carriage fork and roof.stl')).translate([96.8,0,16])
P[w]+=box([81.8,20.0,40.4],[92.2,25.4,62.0]);P[w]-=box([80,25.4,40.0],[92.4,40,62.2])
# Straight ramp followed by roller rounding onto a high dwell. The roller determines lift.
r=RADIUS;slope=SLOPE;closed_z=CLOSED_Z;high=HIGH;intercept=INTERCEPT;crest=CREST
cam=xz([[32,38.5],[78,38.5],[82,40.4],[92.2,40.4],[92.2,62],[82,62],[42,(intercept-slope*42)],[crest,high],[32,high]],25.6,31.0)
for z in [44.4,58.0]:
 P[w]-=cy(2.45,-5,25.5,1,[86,0,z])+cy(3.2,24.6,25.5,1,[86,0,z])
 cam-=cy(2.45,25.5,31.1,1,[86,0,z])+cy(3.2,25.5,26.4,1,[86,0,z])
P[w]+=box([82.8,25.3,48],[90.8,27.8,54.4]);cam-=box([82.6,25.5,47.8],[91,28,54.6])
# The compact bolt guide grows from the existing fixed bearing cheek; no independent tower.
n='Memory — Front bearing cheek';P[n]=solid(trimesh.load(SRC/'Front bearing cheek.stl'))
neck=box([35.8,1.8,36.5],[44.2,5.8,39])+box([32,1.8,38.5],[44.2,11.4,53.3])
neck-=cy(3.3,1.7,5.9,1,[40,0,34.2])
guide=box([32,11.4,45.7],[44.2,25.2,53.3])-box([35.65,16.8,45.6],[40.35,22.4,53.4])
P[n]+=neck+guide
bolt=box([36.05,17.2,40.9],[39.95,22.0,53.4])+box([32,18,53.3],[44,25.2,61.3])
bolt+=(box([36.45,17.6,39.9],[39.55,21.6,39.91])+box([36.05,17.2,40.89],[39.95,22.0,40.91])).hull()
bolt-=cy(2.6,17.9,25.3,1,[38,0,57.3])
# Two small integral band shoulders on the same guide/head, with no remote arms.
def band_lug(z):return cy(3.2,29,30,0,[0,18.6,z])+cy(2.4,30,31.2,0,[0,18.6,z])+cy(3.2,31.2,33,0,[0,18.6,z])
P[n]+=band_lug(49.3);bolt+=band_lug(57.3)
# One rear backbone joins the existing bearing stations. Remove all crank-support scaffolding.
base=solid(trimesh.load(R/'Superseded planar parts'/'Memory — Common baseboard.stl'))^box([-50,0,-40],[50,50,18])
base+=(solid(trimesh.load(R/'Superseded planar parts'/'Write — Common baseboard.stl'))^box([50,0,-20],[150,50,34]))+box([32.2,30.4,-10.7],[66.8,38.4,-2.7])
left=solid(trimesh.load(SRC/'Left side frame.stl')).translate([96.8,0,16]);left-=box([64,-5,-12],[74,26.8,9.8]);base+=left
for z in [-2.4,26]:base+=cy(2.8,22.8,38.4,1,[68.8,0,z])
for name,s,bank,mo in [('Write — Direct cam plate',cam,'Write','carriage'),('Direct lock bolt',bolt,'Lock','bolt'),('Unified rear backbone',base,'Lock','fixed')]:P[name]=s;meta.append(dict(id=name,bank=bank,motion=mo,source='direct cam redesign'))
for p in meta:
 t=mesh(P[p['id']]);assert len(t.split())==1,(p['id'],[c.volume for c in t.split()]);t.export(O/(p['id']+'.stl'),file_type='stl_ascii');p.update(bounds=t.bounds.tolist(),watertight=bool(t.is_watertight),solids=1)
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2))
hs=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];arr={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hs}
def template(n):
 a=arr[n];return a-(a.min(0)+a.max(0))/2
bush=template('Memory — A-input-bush-10')[:,[1,0,2]];axle=template('Memory — A-idler-shaft')[:,[1,0,2]];axle[:,1]*=16/np.ptp(axle[:,1]);pin=template('Write link friction pin 86')
hs=[h for h in hs if not h['id'].startswith(('Lock ','Write link friction pin'))]
def hw(n,a,mo,part,bank='Lock'):
 hs.append(dict(id=n,bank=bank,motion=mo,kind='native',lego_part=part,color=[80,86,91],source='direct cam'));arr[n]=a
hw('Cam roller 2L axle',axle+[38,21.6,57.3],'bolt','32062')
for y in [15.6,27.6]:hw('Cam roller half bush '+str(y),bush+[38,y,57.3],'bolt','32123a')
for z in [44.4,58.0]:hw('Cam attachment friction pin '+str(z),pin+[86,25.5,z],'carriage','2780','Write')
a=[]
for h in hs:h.update(offset=sum(x.size for x in a),vertices=len(arr[h['id']]));a.append(arr[h['id']])
(O/'hardware.json').write_text(json.dumps(hs,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(a))
(O/'Direct cam parameters.json').write_text(json.dumps(dict(roller_radius=r,slope=slope,closed_roller_z=closed_z,high_surface_z=high,intercept=intercept,crest_x=crest,roller_x=38,cam_Y=[25.6,31.0],attachment_pins=[[86,25.5,z] for z in [44.4,58]],guide_integrated_in='Memory — Front bearing cheek',printed_parts=len(meta),hardware_parts=len(hs)),indent=2))
print('Direct cam:',len(meta),'printed parts;',len(hs),'LEGO pieces; crest',crest)
