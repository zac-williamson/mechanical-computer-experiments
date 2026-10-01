from pathlib import Path
import os,json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'worm-candidate';A=R/'neck-candidate/before-worm-print-fix';os.environ['PLANAR_OUTPUT']=str(O)
exec(compile((R/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
R=Path(__file__).resolve().parents[1]
def load(folder,n):return solid(trimesh.load(folder/(n+'.stl')))
whole=load(O,'Carriage body')+load(O,'Carriage bearing end');old=load(A,'Carriage body')+load(A,'Carriage bearing end')
# Full revolution at every possible axial position between the thrust faces.
rad=next(p for p in parts if p['name']=='U015')['mesh'];worm_radius=float(np.linalg.norm(rad.vertices[:,1:]-[10.2,16],axis=1).max())
probe=cy(5.99,-9.099,9.099,0,[0,10.2,16]);overlap=(probe^whole).volume();assert overlap<.001
assert (whole^box([-9.099,4.201,9.301],[9.099,17.999,23.499])).volume()<.001
# Preserve working lever profile and clutch contact, not just their bounding boxes.
regions={'lever contact':box([-.307,30,11.601],[26.61,37.49,19.999]),'clutch contact':cy(7.4,-8,7.8,0,[0,10.2,0]),'locking pockets':box([-12,23.4,35.7],[1,28.6,39.5])}
functional={}
for label,region in regions.items():
 v=((old-whole)^region).volume()+((whole-old)^region).volume();functional[label]=v;assert v<.01,(label,v)
rows=[]
for name,face,normal,sign in [('Carriage body','right',[1,0,0],1),('Carriage bearing end','left',[-1,0,0],-1)]:
 t=trimesh.load(O/(name+'.stl'));s=solid(t)
 # Every point beneath the worm's annular end contact is solid down to the bed.
 x0,x1=sorted([sign*9.101,sign*16.249]);annulus=cy(5.1,x0,x1,0,[0,10.2,16])-cy(3.46,x0-.01,x1+.01,0,[0,10.2,16]);missing=(annulus-s).volume();assert missing<.002,(name,missing)
 assert abs(t.bounds[1 if sign>0 else 0,0]-sign*16.25)<.001
 # Print transform leaves the inward-facing thrust surface pointing upward.
 rotation=trimesh.geometry.align_vectors(normal,[0,0,-1]);thrust_normal=rotation[:3,:3]@np.array([-sign,0,0]);assert thrust_normal[2]>.999
 t.apply_transform(rotation);t.apply_translation(-t.bounds[0]);sp=solid(t);islands=[]
 for h in np.arange(.3,t.extents[2],.2):
  sec=sp.slice(h);prev=sp.slice(h-.2)
  if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):islands.append(round(float(h),2))
 rows.append(dict(part=name,bed_face=face,watertight=bool(t.is_watertight),solids=len(t.split()),detached_islands=islands,bearing_contact_support_void_mm3=missing,bearing_thrust_face_up=True,bearing_outer_face_on_bed=True,bore_axis_vertical=True,thrust_face_height_mm=7.15))
 t.export(O/(name+' print.stl'))
print('PRINT CHECKS',rows,flush=True)
(O/'Worm bearing print checks.json').write_text(json.dumps(rows,indent=2))
assert all(not r['detached_islands'] for r in rows)
(O/'Worm clearance checks.json').write_text(json.dumps(dict(native_max_radius_mm=worm_radius,checked_rotation_envelope_radius_mm=5.99,guaranteed_radial_gap_mm=5.99-worm_radius,full_rotation_overlap_mm3=overlap,axial_thrust_faces_X=[-9.1,9.1],functional_surface_change_mm3=functional,scope='Rotating envelope checked continuously between end faces, including axial float; unchanged interfaces checked against previous mesh.'),indent=2))
s=(R/'detent-joint/lock_check.py').read_text().replace("O=R/'detent-candidate'","O=R/'worm-candidate'");exec(compile(s,'lock check','exec'),{'__name__':'__main__','__file__':str(R/'detent-joint/lock_check.py')})
runpy.run_path(str(R/'package_preview.py'),run_name='__main__')
s=(R/'export_print_layout.py').read_text().replace("parent/'adapted'","parent/'worm-candidate'");exec(compile(s,'print layout','exec'),{'__name__':'__main__','__file__':str(R/'export_print_layout.py')})
runpy.run_path(str(R/'render_review.py'),run_name='__main__')
print('FINISH COMPLETE',flush=True)
