"""Inventory actual axle-bearing parts and check whether their rings start on bed.

This distinguishes a vertical bore from a bore suspended above the print bed.
Other overhangs are measured and reported, not waived as bridgeable.
"""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Wall register';D=O/'Print oriented axle parts';D.mkdir(exist_ok=True)
P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
A={p['id']:V[p['offset']//3:p['offset']//3+p['vertices']] for p in P}
def solid(a):
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
S={p['id']:solid(A[p['id']]) for p in P if p['kind']=='printed'}
def cyl(r,lo,hi,axis,c):
 t=m.Manifold.cylinder(hi-lo,r,circular_segments=32)
 if axis==0:t=t.rotate([0,90,0])
 if axis==1:t=t.rotate([-90,0,0])
 pos=np.array(c).copy();pos[axis]=lo;return t.translate(pos)
axles={'4519','3705','3706','3707','3708','3737','60485','50450','32062','24316','44294','59443','32073'}
H={}
for p in P:
 if p.get('lego_part') not in axles:continue
 a=A[p['id']];lo,hi=a.min(0),a.max(0);axis=int(np.argmax(hi-lo));c=(lo+hi)/2
 for n,s in S.items():
  b=A[n];bl,bh=b.min(0),b.max(0);cross=[j for j in range(3) if j!=axis]
  if any(c[j]<bl[j]-4 or c[j]>bh[j]+4 for j in cross):continue
  l,h=max(lo[axis],bl[axis]),min(hi[axis],bh[axis])
  if h-l<.5:continue
  ring=cyl(3.5,l,h,axis,c)-cyl(2.7,l-.01,h+.01,axis,c)
  if (ring^s).volume()>5:H.setdefault(n,[]).append(dict(axle=p['id'],axis=axis,centre_mm=c.tolist()))
# Include all scheduled transmission bearings, including short axle ends that
# do not reach the newly extended print feet.
beds=json.loads((O/'Bearing bed-face schedule.json').read_text())
wall_names=set()
for w in beds['walls']:
 wall_names.add(w['part']);H[w['part']]=[dict(axle=b['shaft'],axis='XYZ'.index(b['axis']),centre_mm=b['centre_mm']) for b in w['bearings']]
# Frame passages are clearance features; axle bearings live in the fixtures.
# Their printability is covered by the independent, current frame overhang test.
frame_report=json.loads((O/'Frame print checks.json').read_text())
assert frame_report['geometry_sha256']==hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
clearance_only={n:H.pop(n) for n in list(H) if 'coordinated chassis' in n}
for f in D.glob('*.stl'):f.unlink()
results=[]
for n,holes in H.items():
 axes=set(h['axis'] for h in holes);assert len(axes)==1,(n,axes)
 axis=next(iter(axes));a=A[n];s=S[n];mesh=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);alternatives=[]
 for sign in [1,-1]:
  bed=float(a[:,axis].min() if sign==1 else a[:,axis].max());checks=[]
  for h in holes:
   c=h['centre_mm'];l,r=(bed,bed+.04) if sign==1 else (bed-.04,bed)
   # Check outer annulus, clear of the relieved bore in extended rings.
   radius=4.65 if n in wall_names else 3.4;inner=3.3 if n in wall_names else 2.9
   probe=cyl(radius,l,r,axis,c)-cyl(inner,l-.01,r+.01,axis,c)
   ratio=(probe^s).volume()/probe.volume()
   checks.append(dict(**h,bed_ring_fraction=float(ratio),on_bed=ratio>.97))
  normal=np.eye(3)[axis]*sign;bad=(mesh.face_normals@normal<-.70712)&((mesh.triangles_center@normal)>bed*sign+.001)
  alternatives.append(dict(sign=sign,bed_plane_mm=bed,holes=checks,holes_on_bed=sum(h['on_bed'] for h in checks),other_unsupported_area_mm2=float(mesh.area_faces[bad].sum())))
 best=max(alternatives,key=lambda x:(x['holes_on_bed'],-x['other_unsupported_area_mm2']))
 passed=best['holes_on_bed']==len(holes)
 T=trimesh.geometry.align_vectors(np.eye(3)[axis]*best['sign'],[0,0,1]);mesh.apply_transform(T);shift=-mesh.bounds[0];mesh.apply_translation(shift)
 path=D/(n+'.stl')
 # Do not provide misleading print-ready orientations for unresolved parts.
 if passed and n not in wall_names and 'coordinated chassis' not in n:mesh.export(path)
 results.append(dict(part=n,axis='XYZ'[axis],**best,axle_bed_face_pass=passed,file=str(path.relative_to(O)) if path.exists() else None,assembly_to_print_rotation=T.tolist(),print_translation_mm=shift.tolist(),watertight=bool(mesh.is_watertight),connected_solids=len(mesh.split())))
retainer_checks=[]
ps={p['id']:p for p in P};schedule=json.loads((O/'Bearing schedule.json').read_text())
for move in beds.get('moved_collars',[]):
 p=ps[move['part']];a=A[p['id']];face=move['contact_face_mm'];gap=face-a[:,0].max()
 shaft=next(row for row in schedule['retention'] if any(x.get('part')==p['id'] for x in row['stops']))['shaft']
 axle_names=next(row['axle_parts'] for row in schedule['bearings'] if row['shaft']==shaft)
 engagement=max(min(A[n][:,0].max(),a[:,0].max())-max(A[n][:,0].min(),a[:,0].min()) for n in axle_names)
 retainer_checks.append(dict(part=p['id'],gap_mm=float(gap),axle_engagement_mm=float(engagement),pass_check=bool(abs(gap-.2)<1e-5 and engagement>=3.99)))
a=A['bit clock pivot bush 40.4'];v=A['bit clock pivot axle'];gap=a[:,1].min()-38.2;engagement=min(a[:,1].max(),v[:,1].max())-max(a[:,1].min(),v[:,1].min())
retainer_checks.append(dict(part='bit clock pivot bush 40.4',gap_mm=float(gap),axle_engagement_mm=float(engagement),pass_check=bool(abs(gap-.2)<1e-5 and engagement>=3.59)))
assert all(x['pass_check'] for x in retainer_checks),retainer_checks
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,moved_retainer_checks=retainer_checks,clearance_only_frame_passages=clearance_only,frame_clearance_print_pass=frame_report['frame_print_geometry_pass'],parts=results,all_axle_bed_faces_pass=all(p['axle_bed_face_pass'] for p in results),unresolved_parts=[p['part'] for p in results if not p['axle_bed_face_pass']],limitations=['An annular bed-contact screen of detected native-axle interfaces, not an exhaustive hole-feature recognizer.','Vertical holes with bed-supported rings do not imply the rest of a part is support-free. Unsupported surface area is reported separately.','Assembly access, material strength and bore fit require separate validation.'])
(O/'Axle print-face audit.json').write_text(json.dumps(report,indent=2));print('Checked',len(results),'parts; unresolved:',report['unresolved_parts'])
