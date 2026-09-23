"""Independent study using ONLY the multiplexer from the pinned latest commit.
No register or other module is an input. Units: mm, N, Nmm, degrees.
"""
from pathlib import Path
import json,hashlib,itertools,math
import numpy as np,trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union
R=Path(__file__).resolve().parents[1]; B=R.parent/'work/register-mux-reference/multiplexer';O=R/'Investigation';O.mkdir(exist_ok=True)
params=json.loads((B/'parameters.json').read_text());meta=json.loads((B/'printed-parts.json').read_text());hm=json.loads((B/'hardware.json').read_text());vv=np.load(B/'hardware.npz')['vertices']
p={x['id']:trimesh.load(B/(x['id']+'.stl')) for x in meta};h={x['id']:trimesh.Trimesh(vv[x['offset']//3:x['offset']//3+x['vertices']],np.arange(x['vertices']).reshape(-1,3),process=True) for x in hm}
rows=json.loads((B/'Switching trace.json').read_text())['frames']; leads=[]
for a,b in zip(rows,rows[1:]):
 if abs(b['g']-a['g'])<1e-9 and abs(b['q']-a['q'])>1e-6 and abs(b['w']-a['w'])<3:leads.append(360*(b['q']-a['q'])/(b['w']-a['w']))
lead=float(np.median(leads));F=200*2*math.pi/lead
manifest={'commit':'5d987d9d05220db57576819dd6835af9f23af557','input_scope':'multiplexer only; no existing register inspected','files':{str(f.relative_to(B)):hashlib.sha256(f.read_bytes()).hexdigest() for f in B.iterdir() if f.is_file()}}
(O/'Provenance.json').write_text(json.dumps(manifest,indent=2))
# Project real tooth geometry; don't substitute a cylinder for a tooth.
def proj(t):
 a=t.triangles[:,:,[0,2]]
 return unary_union([Polygon(v) for v in a if abs(np.linalg.det(np.array([v[1]-v[0],v[2]-v[0]])))>1e-8])
gear=proj(h['U022']);lever=proj(p['Short lever']); pv=np.array(params['pivot']); gc=np.array(params['reaction_center'])
# Eight-tooth wheel: inspect the lever's actual first tooth root and face breadth.
section=p['Short lever'].section(plane_origin=[0,10.2,0],plane_normal=[0,1,0])
svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="-8 -44 40 30"><rect x="-8" y="-44" width="40" height="30" fill="#faf8f2"/>']
for g,c,n in [(gear,'#93848e','LEGO reaction gear'),(lever,'#dfab39','Unmodified printed lever')]:
 for z in getattr(g,'geoms',[g]):
  xy=np.array(z.exterior.coords); points=' '.join(f'{x:.4f},{-z:.4f}' for x,z in xy);svg.append(f'<polygon points="{points}" fill="{c}" stroke="#223" stroke-width=".05"/>')
svg.append('</svg>');(O/'tooth-profiles.svg').write_text(''.join(svg))
# Continuous-rotation axial withdrawal: conservative AABB proves separation after this travel.
lgap=float(h['U022'].bounds[1,1]-p['Short lever'].bounds[0,1]);withdraw=lgap+.6
# Selection motion: distinguish supplied arbitrary initial pose from reached endpoints.
ends=[]
for i in range(1,len(rows)):
 if rows[i-1]['moving'] and not rows[i]['moving']:ends.append({k:rows[i][k] for k in ['q','b','g','segment']})
# Force/stiffness screening. Material properties below are DESIGN ASSUMPTIONS, not TDS allowables.
loads=[]
for eta in [.2,.4,.6,.8,1.]:
 force=eta*F
 loads.append({'efficiency_assumption':eta,'worm_axial_N':force,'reaction_torque_Nm':force*.004,'lever_tooth_tangential_N_at_4mm':force,'side_release_friction_N_mu_0_35':force*.35})
# Bolt across a keeper, supported at both ends (double shear); beam central force F.
# Window pitch <= endpoint travel restricts bolt width + clearances + web.
qs=[x['q'] for x in rows];travel=min(x['q'] for x in rows if x['q']>3.7)-max(x['q'] for x in rows if x['q']< -3.7)
# Use settled repeated endpoints conservatively from actual reached trace, not intermediate >3.7.
pitch=3.749041-(-3.755874)
solutions=[]
for width in np.arange(2.,7.21,.4):
 for depth in np.arange(4.,16.01,.4):
  for span in [4.,6.,8.,10.]:
   side_clearance=.4; web=pitch-width-2*side_clearance
   sigma=1.5*F*span/(depth*width**2) # bending about depth axis; X force, width X, depth Z
   shear=F/(2*width*depth)
   inertia=depth*width**3/12
   deflection=F*span**3/(48*1800*inertia)
   if web>=2.4 and sigma<=12 and shear<=6 and deflection<=.15:
    solutions.append(dict(width_X=float(width),depth_Z=float(depth),span_Y=span,web_X=web,bending_MPa=sigma,double_shear_MPa=shear,deflection_mm=deflection,area=width*depth))
solutions.sort(key=lambda a:(a['area'],a['span_Y']))
# Same geometry single-supported catch (common compact but weak alternative).
report={'units':'mm,N,Nmm','lead_from_trace_mm_rev':lead,'lead_samples':len(leads),'load_formula':'F=2*pi*T/lead; ideal frictionless screening, not measured load','ideal_axial_N_at_0_2Nm':F,'load_sensitivity':loads,'trace_q_extrema':[min(qs),max(qs)],'trace_reached_release_events':ends,'settled_lock_pitch_mm':pitch,'axial_lever_withdrawal_mm_for_0_6mm_gap':withdraw,'withdrawal_note':'Whole lever must shift >8.2mm in +Y to clear reaction gear in every angular pose; current pivot retainer and rear cheek obstruct this. Not a drop-in modification.','design_allowables_assumed':{'bending_MPa':12,'shear_MPa':6,'modulus_MPa':1800,'side_clearance_mm':.4,'minimum_web_mm':2.4},'double_supported_bolt_candidates':solutions,'best_area_candidate':solutions[0] if solutions else None,'qualification':'Analytical screening only. No certified LEGO or PLA component load capacity, friction, wear or dynamic shock result.'}
(O/'Mechanism study.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['double_supported_bolt_candidates','trace_reached_release_events']},indent=2))
