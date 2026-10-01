from pathlib import Path
import os,json,runpy
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];O=R0/'open-carriage-candidate';A=R0/'neck-candidate/before-open-carriage';os.environ['PLANAR_OUTPUT']=str(O)
exec(compile((R0/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
def load(folder,n):return solid(trimesh.load(folder/(n+'.stl')))
whole=load(O,'Carriage body')+load(O,'Carriage bearing end');old=load(A,'Carriage body')+load(A,'Carriage bearing end')
regions={'lever contact':box([-.307,30,11.601],[26.61,37.49,19.999]),'clutch contact':cy(7.4,-8,7.8,0,[0,10.2,0]),'locking pockets':box([-12,23.4,35.7],[1,28.6,39.5])}
for x in [-11,11]:regions['Rod pin surround '+str(x)]=cy(4.49,15.11,22.39,1,[x,0,32])
for xa,xb in [(-16.24,-9.11),(9.11,16.24)]:regions['Reinforced root '+str(xa)]=box([xa,13.5,24.81],[xb,22.39,27.69])
functional={}
for label,region in regions.items():
 v=((old-whole)^region).volume();functional[label]=v;assert v<.01,(label,v)
assert (whole^box([-9.099,4.201,9.301],[9.099,17.499,23.499])).volume()<.002
rows=[];meshes=[];cursor=0
for name,normal,sign in [('Carriage body',[1,0,0],1),('Carriage bearing end',[-1,0,0],-1)]:
 t=trimesh.load(O/(name+'.stl'));s=solid(t);x0,x1=sorted([sign*9.101,sign*16.249]);annulus=cy(5.1,x0,x1,0,[0,10.2,16])-cy(3.46,x0-.01,x1+.01,0,[0,10.2,16]);missing=(annulus-s).volume();assert missing<.002
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);sp=solid(t);islands=[]
 for h in np.arange(.3,t.extents[2],.2):
  sec=sp.slice(h);prev=sp.slice(h-.2)
  if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):islands.append(round(float(h),2))
 rows.append(dict(part=name,watertight=bool(t.is_watertight),solids=len(t.split()),detached_islands=islands,bearing_support_void_mm3=missing,bore_axis_vertical=True,contact_face_up=True));t.export(O/(name+' print.stl'));t.apply_translation([cursor,0,0]);cursor+=t.extents[0]+8;meshes.append(t)
print('PRINT',rows,flush=True);assert all(not r['detached_islands'] for r in rows)
layout=trimesh.util.concatenate(meshes);layout.export(O/'Carriage print layout.stl');assert layout.is_watertight and len(layout.split())==2
(O/'Open carriage functional checks.json').write_text(json.dumps(dict(preserved_surface_removal_mm3=functional,print_checks=rows,front_extent_Y=-6.4,extra_depth_mm=7.95,carriage_print_layout_dimensions_mm=layout.extents.tolist()),indent=2))
s=(R0/'detent-joint/lock_check.py').read_text().replace("O=R/'detent-candidate'","O=R/'open-carriage-candidate'");exec(compile(s,'lock check','exec'),{'__name__':'__main__','__file__':str(R0/'detent-joint/lock_check.py')})
runpy.run_path(str(R0/'package_preview.py'),run_name='__main__')
s=(R0/'export_print_layout.py').read_text().replace("parent/'adapted'","parent/'open-carriage-candidate'");exec(compile(s,'print layout','exec'),{'__name__':'__main__','__file__':str(R0/'export_print_layout.py')})
runpy.run_path(str(R0/'render_review.py'),run_name='__main__');print('FINISH COMPLETE',flush=True)
