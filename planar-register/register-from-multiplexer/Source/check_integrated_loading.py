"""Conditional statics and rigid-clearance screen, not 0.1 Nm qualification."""
from pathlib import Path
import json,numpy as np
from scipy.optimize import brentq,linprog
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';p=json.loads((O/'Integrated cam parameters.json').read_text())
def lift(q):
 u=p['bolt_x']-q;c=p['crest'];t=c+3.6*1.2/np.sqrt(2.44)
 z=58.7 if u<=c else 55.1+np.sqrt(max(0,12.96-(u-c)**2)) if u<=t else p['intercept']-1.2*u+3.6*np.sqrt(2.44)
 return max(0,z-53.3)
def slope(q):
 u=p['bolt_x']-q;c=p['crest'];t=c+3.6*1.2/np.sqrt(2.44)
 return 0 if u<=c else (u-c)/np.sqrt(12.96-(u-c)**2) if u<t else 1.2
# Conservative guide load calculation credits only the upper guide (X span17.4–18mm, Y span14mm).
# The lower guide constrains tip play but is not credited with sharing cam load.
def normals(V,B,q):
 H=slope(q)*V;low=max(52,49.6+lift(q));h=53.3+lift(q)-low;L=70-low
 rx=-h*H/L;ry=(6*V-11.4*B)/14
 return abs(rx)+abs(-H-rx)+2*abs(ry)
rows=[];play=[]
for q in np.linspace(-3.75,3.75,751):
 u=lift(q);forces=[]
 for mu in [0,.1,.2,.25,.3,.35,.4]:
  def f(V):return V-2-mu*normals(V,2,q)
  knots=sorted(set([2,11.4*2/6,1e6]));sol=None
  for a,b in zip(knots,knots[1:]):
   if f(a)>=0:sol=a;break
   if f(a)*f(b)<=0:sol=float(brentq(f,a,b));break
  forces.append(dict(mu=mu,vertical_N=sol,horizontal_N=None if sol is None else sol*slope(q)))
 rows.append(dict(q=float(q),lift=u,mu_limit=float(1/normals(1,0,q)) if normals(1,0,q)>0 else None,forces=forces))
 # Linearised rigid tilt/translation bounded by both guide lands, X direction.
 low=max(46,42.9+u);high=min(49.2,49.7+u);z=[low,high,56,70];clear=[.3,.3,.4,.4]
 A=[];b=[]
 for zz,cc in zip(z,clear):A.extend([[1,zz],[-1,-zz]]);b.extend([cc,cc])
 tip=42.1+u;roller=53.3+u
 def maxshift(at):return -linprog([-1,-at],A_ub=A,b_ub=b,bounds=[(None,None),(None,None)],method='highs').fun
 play.append(dict(q=float(q),lift=u,tip_X_play_mm=maxshift(tip),roller_X_play_mm=maxshift(roller)))
# Gross variable-section beam. Root at original carriage roof end; plate is integral.
beam=[]
for row in rows:
 q=row['q'];u=p['bolt_x']-q;k=slope(q);xc=u-3.6*k/np.sqrt(1+k*k);zc=53.3+lift(q)-3.6/np.sqrt(1+k*k)
 root=p['write_x']+26.6;xs=np.linspace(root,xc,1001);tops=np.minimum(55.1,p['intercept']-1.2*xs);bottom=np.interp(xs,[root,-25,-20,-1],[40,40,46.5,46.5]);h=tops-bottom;I=5.4*h**3/12
 for force in row['forces']:
  if force['vertical_N'] is None:continue
  F=force['vertical_N'];H=force['horizontal_N'];M=F*(xc-xs)+H*np.abs(zc-(tops+bottom)/2)
  beam.append(dict(q=q,mu=force['mu'],force_N=F,stress_MPa=float(np.max(M*h/(2*I))),deflection_mm=float(np.trapezoid(M*(xc-xs)/(1800*I),xs))))
r=dict(status='Conditional design screen; physical print and 0.1 Nm tests required',assumptions=['Effective modulus 1800 MPa assumed, not measured.','Ideal rolling follower; no tooth impact, inertia, layer anisotropy, creep, or local contact stress.','2 N vertical band force at rear anchor; upper guide span14mm only credited for lifting friction.','Coulomb guide friction moments about transverse face offsets omitted; coefficient sweep is not a measured friction value.','Rigid tilt uses small-angle approximation and nominal clearances; no print error or frame flex.'],guide_overlap_mm=14,band_return_mu_limit=14/(2*11.4),minimum_lift_asymptotic_mu_limit=min(r['mu_limit'] for r in rows if r['mu_limit']),max_tip_X_play_mm=max(x['tip_X_play_mm'] for x in play),max_roller_X_play_mm=max(x['roller_X_play_mm'] for x in play),nominal_keeper_X_margin_at_end_mm=(5.8-3.9)/2-.2,force_summary=[dict(mu=mu,max_vertical_N=max(r['forces'][i]['vertical_N'] or 0 for r in rows),has_no_solution=any(r['forces'][i]['vertical_N'] is None for r in rows),max_gross_cam_deflection_mm=max(x['deflection_mm'] for x in beam if x['mu']==mu),max_gross_cam_stress_MPa=max(x['stress_MPa'] for x in beam if x['mu']==mu)) for i,mu in enumerate([0,.1,.2,.25,.3,.35,.4])],force_sweep=rows,play_sweep=play)
(O/'Integrated loading screen.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k not in ['force_sweep','play_sweep']},indent=2))
