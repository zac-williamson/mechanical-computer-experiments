"""Rigid-contact statics and clearance screen. Not measured friction or FEA."""
from pathlib import Path
import json,math,hashlib,numpy as np
from scipy.optimize import brentq
from direct_cam_math import cam_lift,CREST,TANGENT,INTERCEPT
R=Path(__file__).resolve().parents[1];O=R/'Cam mechanism audit';O.mkdir(exist_ok=True)
def slope(q):
 u=38-q
 if u<=CREST:return 0.
 if u<TANGENT:return (u-CREST)/math.sqrt(3.6**2-(u-CREST)**2)
 return 1.8
# Stem overlap, not the nominal guide block height, is the available bearing span.
def geometry(q):
 lift=cam_lift(q);lower=max(45.7,40.9+lift);upper=min(53.3,53.4+lift)
 return lift,lower,upper,upper-lower,57.3+lift-lower,slope(q)
def normals(V,B,q,F=0):
 lift,lower,upper,L,h,k=geometry(q)
 # F = horizontal residual keeper load on bolt at Z43, signed in X.
 # Roller force acts at Y27.6, guide midplaneY19.6. Band at X30.6,Y18.6.
 H=k*V;My=h*H-7.4*B+(43-lower)*F;Mx=8*V+B
 rxupper=-My/L;rxlower=-H-F-rxupper
 ryupper=Mx/L;rylower=-ryupper
 return abs(rxupper)+abs(rxlower)+abs(ryupper)+abs(rylower)
def required_V(mu,B,q,F=0):
 def residual(V):return V-B-mu*normals(V,B,q,F)
 # Piecewise-linear residual. Scan breakpoints rather than assuming monotonicity.
 _,lower,_,L,h,k=geometry(q);c=-7.4*B+(43-lower)*F
 knots=[B,1e7]
 if h*k:knots.append(-c/(h*k))
 if k*(h/L-1):knots.append((F-c/L)/(k*(h/L-1)))
 knots=sorted(set(x for x in knots if B<=x<=1e7))
 for a,b in zip(knots,knots[1:]):
  if residual(a)>=0:return a
  if residual(a)*residual(b)<=0:return float(brentq(residual,a,b))
 return None
rows=[]
for q in np.linspace(-3.75,3.75,751):
 lift,low,high,L,h,k=geometry(float(q));coef=normals(1,0,float(q));rows.append(dict(q=float(q),lift=lift,guide_overlap=L,roller_above_guide_top=h-L,slope=k,asymptotic_mu_limit=1/coef,forces=[dict(mu=mu,vertical_N=required_V(mu,2,float(q))) for mu in [0,.05,.1,.15,.2,.3,.4]]))
worst=min(rows,key=lambda r:r['asymptotic_mu_limit'])
# Nominal stem bending below guide from lateral keeper load, simple fixed cantilever.
E=1800.;I=4.8*3.9**3/12;L=45.7-39.9
stem=[dict(force_N=F,stress_MPa=F*L*1.95/I,deflection_mm=F*L**3/(3*E*I)) for F in [2,10,50,100,200]]
# Band alone can fail to pull a freely lifted bolt down: off-axis pull reacts at guide ends.
return_rows=[]
for q in [-3.75,-.75,0,1,3.75]:
 lift,low,high,L,h,k=geometry(q);return_rows.append(dict(q=q,lift=lift,overlap=L,band_only_mu_limit=L/(2*(7.4+1))))
r=dict(status='REJECT current short off-axis guide for a confidence print; revise before integrated test',source_sha256={n:hashlib.sha256((R/'Planar register'/n).read_bytes()).hexdigest() for n in ['Direct lock bolt.stl','Memory — Front bearing cheek.stl','Write — Direct cam plate.stl']},assumptions=['Rigid rectangular guide with Coulomb sliding at X/Y walls; reactions separated by available stem overlap.','Ideal freely rolling follower, no cam sliding friction; real bush drag can add load.','Band force2N vertical at X30.6,Y18.6; roller resultant applied at X38,Y27.6.','Guide does not deform; corner pressure, pin compliance and PLA creep excluded.','Friction values are parameter sweeps, not measured PLA coefficients. Failure of this simplified screen is not a prediction of a unique real jam load.'],minimum_overlap_mm=min(x['guide_overlap'] for x in rows),worst_asymptotic_friction_limit=worst,force_sweep=rows,band_return=return_rows,stem_bending=stem,previous_friction_screen='The old 1/(1-1.8*mu) expression omitted the roller overhang and rear offset, and the off-axis return band. It is not a valid qualification of this guide.',minimum_mu_with_no_lift_solution=next((mu for mu in [.05,.1,.15,.2,.3,.4] if any(required_V(mu,2,x['q']) is None for x in rows)),None))
r['assumptions'].append('Guide friction-force moments about transverse face offsets are omitted; the binding threshold is an engineering screen, not an exact contact solution.')
f=json.loads((R/'Planar register/Flex screening.json').read_text());unit=f['load_cases'][0];r['cam_flex_with_guide_amplification']=[]
for c in worst['forces']:
 if c['vertical_N'] is not None:
  scale=c['vertical_N']/unit['vertical_contact_force_N'];r['cam_flex_with_guide_amplification'].append(dict(assumed_mu=c['mu'],vertical_force_N=c['vertical_N'],horizontal_force_N=1.8*c['vertical_N'],cam_gross_beam_stress_MPa=unit['maximum_nominal_bending_MPa']*scale,cam_deflection_at_assumed_E1800_mm=unit['vertical_deflection_mm']*scale))
(O/'Loaded guide screen.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k not in ['force_sweep','source_sha256']},indent=2))
