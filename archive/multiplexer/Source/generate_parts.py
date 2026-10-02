"""Incremental edit of the hash-verified last delivered two-piece design."""
from geometry import *
from clean_print_mesh import clean
import hashlib,gzip,base64,re,copy
import os
R=Path(__file__).resolve().parent;O=R/'construction-inputs'
N=Path(os.environ.get('MUX_BUILD_OUTPUT',R.parent));N.mkdir(parents=True,exist_ok=True)
for entry in json.loads((O/'manifest.json').read_text()):
 assert hashlib.sha256((O/entry['file']).read_bytes()).hexdigest()==entry['sha256'],entry['file']
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
 n=p['id'];source=O/(n+'.stl')
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
   fork=solid(trimesh.load(O/'Clutch stock.stl'))^box([-8,0,-10],[8,20,7.5])
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

print("Generated",len(P),"current printed parts in",N)
