"""Signal-level simplification, after corrected memory drive, before gear phasing."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Planar register';B=R.parent/'work/register-mux-reference/multiplexer'
hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];a={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hs}
# Reverse stored-state assignment by exchanging complete power paths, not by adding gears.
for h in hs:
 if h['id'].startswith(('Memory — A-','Memory — B-')):
  a[h['id']][:,0]*=-1
  a[h['id']]=a[h['id']].reshape(-1,3,3)[:,[0,2,1]].reshape(-1,3).copy()
# Engage the opposite face of the orange clutch. Positive WRITE now selects D.
a['Write — L102']=a['Memory — L072'].copy()+[96.8,0,16]
for h in hs:
 n=h['id']
 if n.startswith('Write — B-') and 'idler' not in n:
  a[n][:,0]=193.6-a[n][:,0];a[n][:,2]+=2.4
  a[n]=a[n].reshape(-1,3,3)[:,[0,2,1]].reshape(-1,3).copy()
  h['axisY']=10.2;h['axisZ']=0
# Remove the original left data-bearing wall and pins; restore the right support and pins.
hs=[h for h in hs if not h['id'].startswith('Write — B-idler') and not h['id'].startswith(('Write — Baseboard pin X-8 ', 'Write — Baseboard pin X-28 '))]
bh=json.loads((B/'hardware.json').read_text());bv=np.load(B/'hardware.npz')['vertices']
for h in bh:
 if h['id'].startswith('Baseboard pin X8 '):
  hh=dict(h);hh.update(id='Write — '+h['id'],source=h['id'],bank='Write');hs.append(hh);a[hh['id']]=bv[h['offset']//3:h['offset']//3+h['vertices']].copy()+[96.8,0,16]
seq=[]
for h in hs:h['offset']=sum(x.size for x in seq);h['vertices']=len(a[h['id']]);seq.append(a[h['id']])
(O/'hardware.json').write_text(json.dumps(hs,indent=2));np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(seq))
def so(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def cy(r,x0,x1,y,z):return m.Manifold.cylinder(x1-x0,r,circular_segments=80).rotate([0,90,0]).translate([x0,y,z])
def tm(s):
 q=s.to_mesh64();return trimesh.Trimesh(np.asarray(q.vert_properties)[:,:3],np.asarray(q.tri_verts),process=False)
meta=json.loads((O/'printed-parts.json').read_text());P={p['id']:so(trimesh.load(O/(p['id']+'.stl'))) for p in meta}
# Memory idler moves from right to left. Leave structural and carriage mounting features in place.
y=10.2+np.sqrt(33.75);z=-10.5
for side,sgn in [('Left',-1),('Right',1)]:
 for kind in ['side frame','inner bearing wall']:
  n=f'Memory — {side} {kind}';s=P[n];inner=kind.startswith('inner');x0,x1=(4.2,11.8) if inner else (24.2,31.8)
  if sgn<0:x0,x1=-x1,-x0
  ca,cb=((-11.8,-7.8) if sgn<0 else (7.8,11.8))
  if side=='Right':
   s+=cy(2.8,x0,x1,y,z)
   if inner:s+=cy(4.05,ca,cb,y,z)
  else:
   s-=cy(2.6,x0-.1,x1+.1,y,z)
   if inner:s-=cy(3.9,ca-.1,cb+.1,y,z)
  P[n]=s
# Use the clean direct-path wall, mirrored, instead of accumulating filled idler bores.
t=trimesh.load(R/'Core layout'/'Memory — Left inner bearing wall.stl');t.vertices[:,0]*=-1;t.faces=t.faces[:,[0,2,1]];P['Memory — Right inner bearing wall']=so(t)
# Restore right inner support; its D bore is at 16 mm centre distance, no idler.
source=next(p for p in json.loads((B/'printed-parts.json').read_text()) if p['id']=='Right inner bearing wall');n='Write — Right inner bearing wall';s=so(trimesh.load(B/'Right inner bearing wall.stl')).translate([96.8,0,16]);P[n]=s;meta.append(dict(source, id=n,bank='Write',source='Right inner bearing wall'))
meta=[p for p in meta if p['id']!='Write — Left inner bearing wall'];del P['Write — Left inner bearing wall']
for n,xa,xb in [('Write — Right inner bearing wall',101,108.6),('Write — Right side frame',121,128.6)]:
 s=P[n];s+=cy(2.8,xa,xb,10.2,-2.4)
 if 'inner' in n:s+=cy(4.05,104.6,108.6,10.2,-2.4)
 s-=cy(2.6,xa-.1,xb+.1,10.2,0)
 if 'inner' in n:s-=cy(3.9,104.6,108.7,10.2,0)
 P[n]=s
# Replace unused orange idler bearing rings with open windows.
for n in ['Write — Right side frame','Write — Right inner bearing wall']:P[n]-=m.Manifold.cube([12,8.4,8]).translate([120 if 'side' in n else 100,14.4,2.8])
# Remove isolated remnants of the deleted idler rings.
for n in ['Write — Right side frame','Write — Right inner bearing wall']:P[n]=max(P[n].decompose(),key=lambda x:x.volume())
# Open the exposed WRITE axle end for a standard 16 mm connector envelope.
for n in ['Write — Right side frame','Write — Rear bearing cheek']:P[n]-=cy(4.5,132.4,150,10.2,32)
for p in meta:
 t=tm(P[p['id']]);t.export(O/(p['id']+'.stl'),file_type='stl_ascii');p.update(bounds=t.bounds.tolist(),watertight=bool(t.is_watertight),solids=len(t.split()))
(O/'printed-parts.json').write_text(json.dumps(meta,indent=2));print('Direct WRITE worm input; one-mesh inverted D; swapped Q power paths. Hardware:',len(hs))
