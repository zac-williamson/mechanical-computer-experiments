"""Conservative static clock-amplifier screening, not structural qualification.

Lossless 0.1 Nm worm input and pi-mm lead give 200 N carriage force. The
2.5:1 lever therefore transmits 80 N. This assumes a blocked sequencer, not
normal running resistance. Separate variable-section Euler-Bernoulli beams for the pin-joined plates estimate
in-plane bending only: no contact/notch stress, torsion, creep, pin compliance,
impact, printed anisotropy or joint slip. Native axle strength is unknown.
"""
from pathlib import Path
import json,hashlib,math
import numpy as np,trimesh
from shapely.geometry import LineString, Point
from shapely.ops import unary_union,polygonize
from shapely.geometry.polygon import orient
R=Path(__file__).resolve().parents[1]/'Compact layout'
ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3)
def mesh(name):
 p=next(p for p in ps if p['id']==name);a=v[p['offset']//3:p['offset']//3+p['vertices']];return trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
def section(t,axis,station,origin_x,component_point=None):
 ij=[i for i in range(3) if i!=axis];c=np.zeros(3);c[axis]=station
 lines=trimesh.intersections.mesh_plane(t,np.eye(3)[axis],c)
 polygons=list(polygonize(unary_union([LineString(np.round(l[:,ij],7)) for l in lines])))
 area=0.;moment=0.;extreme=0.
 for p in polygons:
  if component_point is not None and not p.covers(Point(component_point)):continue
  q=p.representative_point();point=c.copy();point[ij]=q.coords[0]
  if not t.contains([point])[0]:continue
  p=orient(p,sign=1);area+=p.area
  for ring in [p.exterior,*p.interiors]:
   a=np.array(ring.coords);a[:,0]-=origin_x;a[:,1]-=a[:,1].mean();b=np.roll(a,-1,axis=0)
   cross=a[:,0]*b[:,1]-b[:,0]*a[:,1]
   moment+=np.sum((a[:,0]**2+a[:,0]*b[:,0]+b[:,0]**2)*cross)/12
   extreme=max(extreme,float(abs(a[:,0]).max()))
 return area,moment,extreme
force_in=200.;force_out=80.;E=1890.
rows=[]
# Treat the joined plates independently; do not assume a rigid composite beam.
for name,length,moment in [
 ('Clock amplifier front and input shoe',15,lambda x:.5*x-2.5*max(x-12,0)),
 ('Clock amplifier rear and output shoe',30,lambda x:x-2*max(x-15,0))]:
 xs=np.linspace(.03,length-.03,301);stress=[];compliance=[];part=mesh(name)
 for x in xs:
  area,I,c=section(part,2,-56+x,40)
  if I<=0:raise ValueError((name,x,area,I))
  unit=moment(x);stress.append(abs(force_out*unit*c/I));compliance.append(unit**2/(E*I))
 rows.append(dict(part=name,sampled_sections=len(xs),maximum_nominal_bending_MPa=max(stress),output_compliance_mm_per_N=float(np.trapezoid(compliance,xs))))
shoes=[]
for name,root,centre,force in [('Clock amplifier front and input shoe',32.001,30,200),('Clock amplifier rear and output shoe',43.801,46,80)]:
 area,I,c=section(mesh(name),1,root,40,[40,-44 if force==200 else -26])
 shoes.append(dict(part=name,section_area_mm2=area,second_moment_mm4=I,force_N=force,nominal_root_bending_MPa=force*abs(root-centre)*c/I,nominal_average_shear_MPa=force/area))
pivot=mesh('Clock amplifier pivot');area,I,c=section(pivot,1,38.1,40)
pivot_rows=[]
for supports in [(37.8,42.2),(36.,44.1),(34.2,46.)]:
 a,b=supports;loads=[(39.,40.),(41.,80.)];ra=sum(f*(b-y)/(b-a) for y,f in loads)
 moments=[ra*(y-a)-sum(f*max(y-yy,0) for yy,f in loads) for y in np.linspace(a,b,501)]
 pivot_rows.append(dict(support_Y_mm=supports,maximum_moment_Nmm=max(abs(x) for x in moments),nominal_bending_MPa=max(abs(x) for x in moments)*c/I))
report=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),input_torque_Nm=.1,lossless_input_force_N=force_in,output_force_N=force_out,printed_plates=rows,integral_shoes=shoes,ideal_relative_output_deflection_mm=force_out*sum(r['output_compliance_mm_per_N'] for r in rows),pivot_support_sensitivity=pivot_rows,joining_pin_shear_N_each=80.,PLA_reference=dict(modulus_MPa=E,source='https://store.bblcdn.com/s7/default/b189de92249a4b9ebed28b8ea1f080f0/Bambu_PLA_Basic_Technical_Data_Sheet.pdf',notes='Supplier specimen properties are not part allowables.'),unresolved=['Cam shoe friction and wear','Pin joint compliance and strength','Notch stress and printed anisotropy','LEGO shaft material allowable','Dynamic and fatigue loads'],mechanically_qualified=False)
(R/'Clock load screening.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
