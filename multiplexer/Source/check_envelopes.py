from geometry import *
O=OUT;meta=json.loads((O/'printed-parts.json').read_text());P={p['id']:solid(trimesh.load(O/(p['id']+'.stl'))) for p in meta};H=json.loads((O/'hardware.json').read_text());V=np.load(O/'hardware.npz')['vertices'];hits=[];pms=json.loads((O/'parameters.json').read_text());pv=np.array(pms['pivot']);qs=np.linspace(-4.6,4.575,185)
for h in H:
 n=h['id'];mo=h.get('motion');v=V[h['offset']//3:h['offset']//3+h['vertices']]
 if mo=='fixed' or n=='L099' or n.startswith('Carriage support pin'):continue
 axis=1 if mo=='gear' else 0;center=np.array([0,10.2,24.0]) if axis==1 else np.array([0,10.2,16.0]) if mo in ['worm','input'] else np.array([0,10.2,0])
 if mo in ['A','B'] and n not in ['L072','L102']:center[2]=-18.4
 if 'idler' in mo:center[1:]=[10.2+np.sqrt(144-9.2**2),-9.2]
 ix=[j for j in range(3) if j!=axis];r=np.linalg.norm(v[:,ix]-center[ix],axis=1).max();s=cyl(float(r),float(v[:,axis].min()),float(v[:,axis].max()),axis,center)
 if n=='reaction-stop-axle':s=cyl(2.4,float(v[:,1].min()),float(v[:,1].max()),1,center)+cyl(3.2,18.8,19.6,1,center)
 for p in meta:
  name=p['id']
  if name=='Short lever':continue
  # Expected clutch guide contact is checked using the native groove mesh, not a filled cylinder.
  for q in (qs if mo=='worm' or p['motion']=='carriage' else [0]):
   ss=s.translate([q,0,0]) if mo=='worm' else s;pp=P[name].translate([q,0,0]) if p['motion']=='carriage' else P[name];vol=(ss^pp).volume()
   if vol>.002:hits.append(dict(hardware=n,printed=name,q=float(q),volume=float(vol)));break
P['Single braced carriage']=P['Carriage fork and roof']+P['Right carriage bearing support']
# Full planar band sweep including .3 mm geometric margin against full carriage travel.
from shapely.geometry import LineString
from shapely import affinity
aa=np.array([37.19232360398867, -28.12844869819761]);bb=np.array([22.192323603988665, -30.62844869819761]);op=np.array([13.192323603988665, -32.12844869819761]);rings=[]
for b in np.linspace(-28.5,28.5,229):
 angle=np.radians(b);rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);bm=rot@(bb-op)+op;l=LineString([aa,bm]);rings.append(l.buffer(3.5,quad_segs=24)-l.buffer(1.7,quad_segs=24))
s=extr(unary_union(rings),0,1.8).rotate([-90,0,0]).translate([0,11.1,0]);bandhits=[]
for q in qs:
 vol=(s^P['Single braced carriage'].translate([q,0,0])).volume()
 if vol>.001:bandhits.append(dict(q=float(q),volume=vol))
# Worm insertion from the open negative-Y side of the one-piece carriage.
worm=cyl(5.,-7.86,7.86,0,(0,10.2,16.0));assembly=[]
for dy in np.linspace(-40,0,161):
 vol=(worm.translate([0,float(dy),0])^P['Single braced carriage']).volume()
 if vol>.001:assembly.append(dict(dy=float(dy),volume=vol))
rail_assembly=[]
for dy in np.linspace(30,0,121):
 vol=(P['Common baseboard'].translate([0,float(dy),0])^P['Single braced carriage']).volume()
 if vol>.001:rail_assembly.append(dict(dy=float(dy),volume=vol))
collar_hits=[]
for hh in H:
 n=hh['id']
 if not n.startswith(('Frame pin','Cartridge','Baseboard pin')):continue
 vv=V[hh['offset']//3:hh['offset']//3+hh['vertices']];axis=1 if n.startswith(('Cartridge','Baseboard pin')) else 0;ix=[k for k in range(3) if k!=axis];ctr=(vv.min(0)+vv.max(0))/2;rr=np.linalg.norm(vv[:,ix]-ctr[ix],axis=1);aa=vv[rr>2.9,axis];cs=cyl(3.2,float(aa.min()),float(aa.max()),axis,ctr)
 for name,s in P.items():
  if name in ['Single braced carriage','Short lever','Left carriage bearing support','Right carriage bearing support','Carriage fork and roof']:continue
  vol=(cs^s).volume()
  if vol>.001:collar_hits.append(dict(pin=n,part=name,volume=vol))
r=dict(pin_collar_interferences=collar_hits,rail_insertion_interferences=rail_assembly,rotating_envelope_interferences=hits,band_carriage_sweep_interferences=bandhits,worm_insertion_interferences=assembly,carriage_positions=len(qs),band_angles=229,notes='Filled full-revolution cylinders are conservative; intended gear/lever and ring/fork contacts excluded. This is geometric screening, not a load test.')
(O/'envelope-check.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
