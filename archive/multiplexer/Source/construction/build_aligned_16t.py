"""Incremental edit of the hash-verified last delivered two-piece design."""
from geometry import *
from clean_print_mesh import clean
import hashlib,gzip,base64,re,copy
R=Path(__file__).resolve().parent;O=R/'Flat-actuator';N=O/'Aligned 16T spacing';N.mkdir(exist_ok=True)
manifest=json.loads((O/'Two-piece exploration'/'Two-piece print layout manifest.json').read_text())
for p in manifest['parts']:
 assert hashlib.sha256(Path(p['source']).read_bytes()).hexdigest()==p['sha256'],p['part']
G=16.;dz=1.5;dy=-5.65
# Insert a straight section, preserving all geometry on either side rigidly.
# Frame/board seam avoids clutch bearing, rear mounting bore and guide.
# Carriage seam lies beyond the tested clutch fork, before the worm bearing.
def extend(s,z,shift_y=0):
 mid=s.slice(z).extrude(dz+.002).translate([0,0,z-.001])
 mid=mid.transform([[1,0,0,0],[0,1,shift_y/dz,-shift_y*z/dz],[0,0,1,0]])
 return (s^box([-100,-100,-100],[100,100,z]))+(s^box([-100,-100,z],[100,100,100])).translate([0,shift_y,dz])+mid
def raise_frame(s):
 # Preserve foot and mounting bores. Stretch the wall below the shifted bearing.
 low=s^box([-100,-100,-100],[100,19.45,100])
 mid=s^box([-100,19.45,-100],[100,22.3,100])
 high=s^box([-100,22.3,-100],[100,100,100])
 scale=(22.3-19.45-dy)/(22.3-19.45)
 return low.translate([0,dy,0])+mid.transform([[1,0,0,0],[0,scale,0,22.3*(1-scale)],[0,0,1,0]])+high
meta=json.loads((O/'printed-parts.json').read_text());meta=[p for p in meta if p['id'] not in ['Left carriage bearing support','Carriage fork and roof']]
meta.append(dict(id='Carriage fork and roof',motion='carriage',print_rotation_axis=[0,1,0],print_rotation_angle=-np.pi/2))
P={};changes=[]
for p in meta:
 n=p['id'];source=O/(n+'.stl') if n!='Carriage fork and roof' else O/'Two-piece exploration'/'Flush underside candidate.stl'
 t=trimesh.load(source);s=solid(t)
 if n in ['Front bearing cheek','Rear bearing cheek','Short lever']:s=s.translate([0,dy,dz]);changes.append(n)
 elif n in ['Left side frame','Right side frame']:
  front=s^box([-100,-100,-100],[100,100,6])
  rear=s^box([-100,-100,6],[100,100,100])
  s=front+raise_frame(rear).translate([0,0,dz])+s.slice(6).extrude(dz+.002).translate([0,0,5.999]);changes.append(n)
 elif n=='Common baseboard':
  s=extend(s,6.0)^box([-100,30.4,-100],[100,100,100])
  s+=box([-23.8,15.55,9.1],[23.8,21.55,12.1])
  s+=box([-20.8,21.55,9.1],[20.8,30.401,12.1])

  for aa,bb in [(-23.8,-20.8),(20.8,23.8)]:s+=box([aa,21.25,9.1],[bb,30.401,12.1])
  changes.append(n)
 elif p['motion']=='carriage':
  original=s
  frontjoint=s^box([-30,20.2,-10],[30,40,5.8])
  if n=='Carriage fork and roof':
   fork=load('Left carriage half')^box([-8,0,-10],[8,20,7.5])
   s=s.translate([0,dy,dz])+box([-1.6,1.55,7.45],[1.6,11.6,9.05])
  else:s=s.translate([0,dy,dz])
  s-=frontjoint.translate([0,dy,dz])
  s+=frontjoint.translate([0,0,dz])
  xa,xb=(8,15.6) if n=='Right carriage bearing support' else (-15.6,-8)
  s+=box([xa,13.5,6.8],[xb,24.4,8.7])+box([xa,13.5,8.6],[xb,15.1,18.5])
  # Preserve the rear pin bore and its collar/chamfer void through the new web.
  void=cyl(3.6,xa-.1,xb+.1,0,(0,29.6,18))-original
  s-=void.translate([0,dy,dz])
  for aa,bb in [(-30,-7.3),(7.3,30)]:s-=cyl(9.15,aa,bb,0,(0,10.2,0))
  s-=cyl(7.6,-7.3,7.3,0,(0,10.2,0))
  if n=='Carriage fork and roof':s+=fork
  changes.append(n)
 t=clean(mesh(s.simplify(0.001)));t.export(N/(n+'.stl'),file_type='stl_ascii');t=clean(trimesh.load(N/(n+'.stl')));t.export(N/(n+'.stl'),file_type='stl_ascii')
 assert t.is_watertight and len(t.split())==1,n
 p.update(watertight=True,solids=[float(t.volume)]);P[n]=solid(t)
(N/'printed-parts.json').write_text(json.dumps(meta,indent=2))
params=json.loads((O/'parameters.json').read_text())
for k in ['worm_center','reaction_center','pivot']:params[k][2]+=dz;params[k][1]+=dy
(N/'parameters.json').write_text(json.dumps(params,indent=2))
h=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];newh=[];arrays=[]
for p in h:
 n=p['id']
 if n.startswith('Carriage support pin -1 '):continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']].copy()
 move=p['motion'] in ['input','worm','gear'] or n.startswith(('pivot-','Cartridge')) or n.startswith('Baseboard pin') and n.endswith('Z10') or n.startswith('Carriage support pin 1 ')
 if move:
  a[:,2]+=dz
  if not n.startswith('Baseboard pin') and n!='Carriage support pin 1 2.0':a[:,1]+=dy
 p['offset']=sum(x.size for x in arrays);arrays.append(a);newh.append(p)
(N/'hardware.json').write_text(json.dumps(newh,indent=2));np.savez_compressed(N/'hardware.npz',vertices=np.concatenate(arrays))
mounts=json.loads((O/'baseboard-mounts.json').read_text())
for p in mounts:
 if p['z']==10:p['z']+=dz
# Keep hardware IDs stable; checker uses the old ID to identify the translated pin.
(N/'baseboard-mounts.json').write_text((O/'baseboard-mounts.json').read_text())
(N/'mount-coordinates.json').write_text(json.dumps(mounts,indent=2))
# Preserve the original viewer and print pack; derive a separate candidate viewer.
html=(O/'Two-piece viewer.html').read_text();match=re.search(r'<script type="application/json" id="data">(.*?)</script>',html,re.S);D=json.loads(match[1]);a0=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4');ar=[]
lookup={p['id']:a for p,a in zip(newh,arrays)}
for p in D['parts']:
 n=p['id'];a=a0[p['offset']:p['offset']+p['vertices']*3].reshape(-1,3).copy()
 if n=='Merged bearing and arm':a=mesh(P['Carriage fork and roof']).triangles.reshape(-1,3)
 elif n in P:a=mesh(P[n]).triangles.reshape(-1,3)
 elif n in lookup:a=lookup[n]
 elif p['motion']=='band':a[:,2]+=dz;a[:,1]+=dy
 p.update(offset=sum(x.size for x in ar),vertices=len(a));ar.append(a)
for k in ['pivot','reaction','bandA','bandB']:D[k][2]+=dz;D[k][1]+=dy
D['bounds'][0][1]-=5.65;D['bounds'][1][2]+=dz;D['geometry']=base64.b64encode(gzip.compress(np.concatenate(ar).astype('<f4').tobytes())).decode()
html=html[:match.start(1)]+json.dumps(D,separators=(',',':'))+html[match.end(1):]
html=html.replace('[15.25,16.45]','[9.60,10.80]').replace('pt(ar[i],15.25)','pt(ar[i],9.60)').replace('pt(ar[j],15.25)','pt(ar[j],9.60)').replace('pt(ar[j],16.45)','pt(ar[j],10.80)').replace('pt(ar[i],16.45)','pt(ar[i],10.80)')
html=html.replace('[0,15.85,14.5]',f'[0,10.2,{G}]').replace('<option value="carriage" selected>','<option value="carriage">')
html=html.replace('[15.25,16.45]','[9.6,10.8]').replace('pt(ar[i],15.25)','pt(ar[i],9.6)').replace('pt(ar[j],15.25)','pt(ar[j],9.6)').replace('pt(ar[j],16.45)','pt(ar[j],10.8)').replace('pt(ar[i],16.45)','pt(ar[i],10.8)')
html=html.replace('Two-piece carriage — candidate','16T-compatible axle spacing').replace('<h1>Two-piece carriage candidate</h1>',f'<h1>16T-compatible axle spacing</h1><p>Axle centre distance: 16.000 mm. Worm and actuator moved {dy:.2f} mm in Y and +{dz:.2f} mm in Z; fork connection extended by the same amount.</p>')
(O/'Aligned 16T spacing.html').write_text(html)
report=dict(target_center_distance_mm=16,old_center_distance_mm=float(np.hypot(14.5,5.65)),worm_z_mm=G,shift_z_mm=dz,shift_y_mm=dy,changed_parts=changes,unchanged_parts=['Left inner bearing wall','Right inner bearing wall'],method='Actuator rigidly translated by [0,-5.65,1.5]. Original clutch fork retained and connected by a longer neck. Front carriage pin kept at original Y; bearing-side ribs route above the open-bottom guide. Frame feet remain at original height.',source_manifest=str(O/'Two-piece exploration'/'Two-piece print layout manifest.json'))
(N/'Change record.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
