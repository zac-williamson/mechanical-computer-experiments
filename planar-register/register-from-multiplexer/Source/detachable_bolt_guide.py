"""Separate printable bolt guide; support-free base geometry and matched pin mounts."""
from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m,shutil
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';meta=json.load(open(O/'printed-parts.json'));BX=-5.05;N='Detachable bolt guide';BASE='Unified rear backbone'
if not (O/'Frame before detachable guide.stl').exists():shutil.copy2(O/(BASE+'.stl'),O/'Frame before detachable guide.stl')
def so(t):return m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True)))
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def cyl(r,z0,z1,x,y,r2=None):return m.Manifold.cylinder(z1-z0,r,r if r2 is None else r2,circular_segments=128).translate([x,y,z0])
def roofhole(r,z0,z1,x,y):
 c=m.CrossSection.circle(r,128).translate([x,y]);tri=m.CrossSection([np.array([[x-r/np.sqrt(2),y-r/np.sqrt(2)],[x+r/np.sqrt(2),y-r/np.sqrt(2)],[x,y-r*np.sqrt(2)]])],m.FillRule.EvenOdd);return (c+tri).extrude(z1-z0).translate([0,0,z0])
def xz(poly,y0,y1):return m.CrossSection([np.array(poly)]).extrude(y1-y0).transform([[1,0,0,0],[0,0,1,y0],[0,1,0,0]])
s=so(trimesh.load(O/'Frame before detachable guide.stl'));base=s^box([-160,-5,-40],[60,45,43.8]);guide=s^box([-160,-5,44.2],[60,45,100])
# Continue the guide rail profiles all the way back to the bed plane.
for x,z in [(0,0),(-85,16)]:base+=box([x-23.8,21.55,z+9.0],[x+23.8,40.4,z+12.2])
mounts=[]
for sign in [-1,1]:
 x=BX+sign*14.5;y=36.1
 base+=box([x-4.3,31.4,36.2],[x+4.3,40.4,43.8])
 # Outside locating shoulder, with 0.3 mm lateral assembly clearance.
 aa,bb=(x-6.2,x-4.6) if sign<0 else (x+4.6,x+6.2)
 base+=box([aa,32.8,36.2],[bb,40.4,48.5])
 # Shoulder is rooted in the whole lug, not merely tangent to it.
 base+=box([min(aa,x-4.3),32.8,36.2],[max(bb,x+4.3),40.4,43.8])
 guide+=box([x-4.3,31.8,44.2],[x+4.3,40.4,51.8])
 guide+=box([min(x,BX+sign*8.2),34.6,44.2],[max(x,BX+sign*8.2),40.4,51.8])
 base-=roofhole(2.5,36.1,43.9,x,y)+roofhole(3.35,43.1,43.9,x,y)
 guide-=roofhole(2.5,44.1,51.9,x,y)
 mounts.append(dict(x=x,y=y,axis='Z',pin_center_z=44,base_Z=[36.2,43.8],guide_Z=[44.2,51.8]))
# Fill rear support shadows to the bed plane, then restore every mounting bore.
shadow=base.rotate([90,0,0]).project().offset(.01).simplify(.0001).extrude(7.61).rotate([-90,0,0]).translate([0,32.79,0])
base=(base^box([-170,-5,-50],[70,32.8,100]))+shadow
for pin in json.load(open(O/'Bearing frame datums.json'))['mounts']:
 for r,a,b in [(2.5,32.7,40.5),(3.35,32.7,33.5)]:base-=m.Manifold.cylinder(b-a,r,circular_segments=128).rotate([-90,0,0]).translate([pin['x'],a,pin['z']])
for pin in mounts:base-=roofhole(2.5,36.1,43.9,pin['x'],pin['y'])+roofhole(3.35,43.1,43.9,pin['x'],pin['y'])
# Rebuild the guide as a flat-footed pair of columns. The working bolt faces
# keep their original positions; only the rear cam opening gets an arched roof.
guide=box([BX-10,14,45.5],[BX+10,40.4,46.2])
guide+=box([BX-10,14,45.5],[BX+10,24.8,49.2])
guide-=box([BX-2.25,17.7,45.4],[BX+2.25,22.3,49.3])
for lo,hi in [(-10,-6.4),(6.4,10)]:
 guide+=box([BX+lo,14,45.5],[BX+hi,25.2,70])+box([BX+lo,31.4,45.5],[BX+hi,40.4,70])
 arch=m.CrossSection([np.array([[25.19,55.99],[28.3,59.1],[31.41,55.99],[31.41,70],[25.19,70]])]).extrude(hi-lo).transform([[0,0,1,BX+lo],[1,0,0,0],[0,1,0,0]])
 guide+=arch
 # Front retaining cheeks have a 45-degree underside and preserve their faces.
 xa,xb=(BX-6.4,BX-4.6) if lo<0 else (BX+4.6,BX+6.4)
 poly=[[xa,50.2 if lo<0 else 52],[xb,52 if lo<0 else 50.2],[xb,70],[xa,70]]
 guide+=xz(poly,14,17.6)
 # Rear lips capture the broad head in +Y. The 45-degree underside grows
 # inward from the existing arch, above the entire cam swept envelope.
 xa,xb=(BX-6.42,BX-4.6) if lo<0 else (BX+4.6,BX+6.42)
 poly=[[xa,59.18 if lo<0 else 61],[xb,61 if lo<0 else 59.18],[xb,70],[xa,70]]
 guide+=xz(poly,25.6,29.2)
for pin in mounts:
 x,y=pin['x'],pin['y']
 guide+=box([x-4.3,31.8,45.5],[x+4.3,40.4,53.1])
 guide+=box([min(x,BX)-.01,34.6,45.5],[max(x,BX)+.01,40.4,49])
 guide-=cyl(2.5,45.4,53.2,x,y)
 # Positive seating lands locate the guide height independently of pin friction.
 for aa,bb in [(x-4.3,x-3.5),(x+3.5,x+4.3)]:
  base+=box([aa,32.8,43.8],[bb,40.4,45.5])
 pin['guide_Z']=[45.5,53.1]
# Fixed elastic anchor raised 2 mm to clear the flat foot. Its saddle flanges
# have full-height feet; only the 1.2 mm band groove is bridged.
for ya,yb,r in [(31.4,32.4,3.2),(32.4,33.6,2.4),(33.6,34.6,3.2)]:
 guide+=m.Manifold.cylinder(yb-ya,r,circular_segments=128).rotate([-90,0,0]).translate([BX,ya,50])
 if r==3.2:guide+=box([BX-3.2,ya,45.5],[BX+3.2,yb,50])
def export(n,s):
 q=s.simplify(1e-5).to_mesh64();t=trimesh.Trimesh(q.vert_properties[:,:3],q.tri_verts,process=True);
 if not t.is_watertight:
  t.merge_vertices(digits_vertex=7);t.update_faces(t.nondegenerate_faces());t.remove_unreferenced_vertices()
 if not t.is_watertight:
  counts=np.bincount(t.edges_unique_inverse);print('Bad edges',t.vertices[t.edges_unique[counts!=2]].tolist()[:12],flush=True)
 assert t.is_watertight and len(t.split())==1,(n,[(c.volume,c.bounds.tolist()) for c in t.split()]);t.export(O/(n+'.stl'),file_type='stl_ascii');return t
for name,solid in [(BASE,base),(N,guide)]:
 t=export(name,solid);p=next((p for p in meta if p['id']==name),None)
 if p is None:p=dict(id=name,bank='Lock',motion='fixed',source='detachable guide');meta.append(p)
 p.update(bounds=t.bounds.tolist(),watertight=True,solids=1)
 if name==N:p.update(print_rotation_axis=[1,0,0],print_rotation_angle=0.0)
hw=json.load(open(O/'hardware.json'));v=np.load(O/'hardware.npz')['vertices'];A={h['id']:v[h['offset']//3:h['offset']//3+h['vertices']].copy() for h in hw};ref=next(h for h in hw if h['id'].startswith('Frame pin '));a=A[ref['id']].copy();a-=(a.min(0)+a.max(0))/2;a=trimesh.transform_points(a,trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]));hw=[h for h in hw if not h['id'].startswith('Guide mount friction pin ')]
for i,pin in enumerate(mounts):
 n=f'Guide mount friction pin {i+1}';A[n]=a+[pin['x'],pin['y'],44];hw.append(dict(id=n,bank='Lock',kind='native',motion='fixed',color=[54,133,180],lego_part=ref.get('lego_part','2780'),vertices=len(a)))
arr=[]
for h in hw:h['offset']=sum(a.size for a in arr);arr.append(A[h['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(arr));(O/'hardware.json').write_text(json.dumps(hw,indent=2));(O/'printed-parts.json').write_text(json.dumps(meta,indent=2));(O/'Detachable guide mounts.json').write_text(json.dumps(dict(mounts=mounts,shoulder_clearance_mm=.3,print_guide_axis='+Z',print_base_axis='-Y',fixed_band_anchor_z=50,guide_seat_z=45.5),indent=2));print('Separated guide and added two LEGO pin joints')
