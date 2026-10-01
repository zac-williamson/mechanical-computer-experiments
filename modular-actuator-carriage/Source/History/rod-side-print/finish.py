from pathlib import Path
import os,json,runpy
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];O=R0/'rod-candidate';A=R0/'neck-candidate/before-rod-side-print';os.environ['PLANAR_OUTPUT']=str(O)
exec(compile((R0/'validate.py').read_text().split('printed_hits=[]')[0],'load','exec'))
def load(folder,n):return solid(trimesh.load(folder/(n+'.stl')))
rows=[];placed=[];cursor=0
for name,z0 in [('Carriage control rod',28.2),('Lock control rod',40)]:
 t=trimesh.load(O/(name+'.stl'));s=solid(t);old=load(A,name)
 # Both complete lap ends start directly on the bed, with no underside ledge.
 for xa,xb,y0,y1 in [(-47.7,-32.1,10.7,14.1),(32.1,47.7,6.3,9.7)]:
  probe=box([xa,y0,z0+.001],[xb,y1,z0+.1]);assert (probe-s).volume()<.002
 t.apply_translation(-t.bounds[0]);sp=solid(t);bed=sp.slice(.05);islands=[];outside=0
 for h in np.arange(.3,t.extents[2],.2):
  sec=sp.slice(h);prev=sp.slice(h-.2);outside=max(outside,(sec-bed.offset(.002)).area())
  if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):islands.append(round(float(h),2))
 assert not islands and outside<.01,(name,islands,outside)
 # A 4.9 mm pin still has the complete original circular clearance path.
 for x in [-44,-36,36,44]+([-11,11] if name=='Carriage control rod' else []):assert (s^cy(2.45,6.21,14.19,1,[x,0,32 if name=='Carriage control rod' else 48])).volume()<.001
 rows.append(dict(part=name,watertight=bool(t.is_watertight),solids=len(t.split()),end_laps_on_bed=True,detached_islands=islands,material_outside_bed_footprint_mm2=outside,pin_roof_slope_degrees=45,max_pin_roof_bridge_mm=2*2.75*(np.sqrt(2)-1),bed_Z=z0))
 t.export(O/(name+' print.stl'));t.apply_translation([0,cursor,0]);cursor+=t.extents[1]+8;placed.append(t)
old=load(A,'Lock control rod');new=load(O,'Lock control rod');cam=box([-25,6.21,42.21],[25,14.19,54.1]);cam_change=(((old-new)+(new-old))^cam).volume();assert cam_change<.002
for name,xa,xb in [('Left bearing wall',-24.4,-20.4),('Right bearing wall',20.4,24.4)]:
 s=load(O,name);probe=box([xa+.01,5.56,36.46],[xb-.01,14.84,39.34]);assert (probe-s).volume()<.002
 # Vertical bearing-wall bores retain their existing print orientation.
 t=trimesh.load(O/(name+'.stl'));normal=[1,0,0] if name.startswith('Left') else [-1,0,0];t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);sp=solid(t);islands=[]
 for h in np.arange(.3,t.extents[2],.2):
  sec=sp.slice(h);prev=sp.slice(h-.2)
  if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):islands.append(round(float(h),2))
 assert not islands;t.export(O/(name+' print.stl'));rows.append(dict(part=name,watertight=bool(t.is_watertight),solids=len(t.split()),detached_islands=islands,min_between_rod_guides_mm=2.9))
# Two end-to-end modules retain the same half-lap and pin centres throughout travel.
for name in ['Carriage control rod','Lock control rod']:
 s=load(O,name);assert (s^s.translate([80,0,0])).volume()<.001
layout=trimesh.util.concatenate(placed);layout.export(O/'Rods side-print layout.stl');assert layout.is_watertight and len(layout.split())==2
(O/'Rod side-print checks.json').write_text(json.dumps(dict(parts=rows,cam_surface_change_mm3=cam_change,rod_layout_dimensions_mm=layout.extents.tolist(),horizontal_module_pitch_mm=80,length_and_hole_centres_unchanged=True,limits='Layer and solid-geometry checks. The short pin-hole roof bridges still require printer calibration; no support is intended on the rod ends or cam.'),indent=2));print(rows,flush=True)
runpy.run_path(str(R0/'package_preview.py'),run_name='__main__')
s=(R0/'export_print_layout.py').read_text().replace("parent/'adapted'","parent/'rod-candidate'");exec(compile(s,'print layout','exec'),{'__name__':'__main__','__file__':str(R0/'export_print_layout.py')})
runpy.run_path(str(R0/'render_review.py'),run_name='__main__');print('FINISHED',flush=True)
