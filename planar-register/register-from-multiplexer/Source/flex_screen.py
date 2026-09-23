"""Conditional beam screen of the direct cam; not material/load qualification."""
from pathlib import Path
import json,numpy as np
from direct_cam_math import INTERCEPT
O=Path(__file__).resolve().parents[1]/'Planar register';E=1800;thickness=5.4
xc=41.75-3.6*1.8/np.sqrt(4.24);zc=57.3-3.6/np.sqrt(4.24);xs=np.linspace(xc,86,1001)
tops=np.where(xs<=42,INTERCEPT-1.8*xs,np.where(xs<82,(INTERCEPT-1.8*42)+(xs-42)*(62-(INTERCEPT-1.8*42))/40,62))
bottoms=np.where(xs<=78,38.5,np.where(xs<82,38.5+(xs-78)*1.9/4,40.4));h=tops-bottoms;I=thickness*h**3/12;centres=(tops+bottoms)/2;rows=[]
for F in [2,10,50,200/1.8]:
 M=F*((xs-xc)+1.8*(zc-centres));stress=np.abs(M)*h/(2*I);deflection=np.trapezoid(M*(xs-xc)/(E*I),xs)
 rows.append(dict(vertical_contact_force_N=F,horizontal_force_N=1.8*F,maximum_nominal_bending_MPa=float(stress.max()),vertical_deflection_mm=float(deflection)))
r=dict(required_axle_torque_Nm=.1,cam_thickness_mm=5.4,minimum_nominal_beam_height_mm=float(h.min()),pressure_angle_degrees=float(np.degrees(np.arctan(1.8))),assumptions=['Effective elastic modulus1800MPa is an assumed screening value.','Cantilever fixed at attachment X86; unperforated variable-depth beam idealisation.','Excludes mounting holes/key recess, joint compliance, stress concentrations, layer anisotropy, local roller contact and creep.','200N horizontal case is the ideal blocked-worm force scale, not a proven transmitted cam load.'],load_cases=rows,changes=['Crank and its three axles removed; one 2L roller axle remains on the bolt.','Cam and roller are in the same plane; the long spaced follower axle is removed.','Guide is integrated into the existing bearing cheek.','Deep continuous cam web replaces the long narrow slotted link.'],verdict='Conditional stiffness comparison only. All-axle0.1Nm survival and physical operation remain unqualified.')
(O/'Flex screening.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
