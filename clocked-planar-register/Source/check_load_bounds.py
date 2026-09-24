"""Beam screening, not FEA or qualification. N, mm, MPa units."""
from pathlib import Path
import json,math
R=Path(__file__).resolve().parents[1]
E=2000.;T=100.;lead=math.pi
worm_force=2*math.pi*T/lead
rail_force=worm_force/3
fork_force=rail_force/(4.85/3)
L=94.;depth=5.4;height=9.7;I=depth*height**3/12
rail=dict(span_mm=L,section_mm=[depth,height],I_mm4=I,
 deflection_at_1N_mm=L**3/(48*E*I),
 stress_at_55N_MPa=55*L/4/(depth*height**2/6),
 deflection_at_55N_mm=55*L**3/(48*E*I))
length=30.;Iy=3.6*(4.5**3+9.6**3)/12
fork=dict(stem_widths_mm=[4.5,9.6],depth_mm=3.6,assumed_free_length_mm=length,
 load_N=fork_force,combined_I_mm4=Iy,
 free_rotation_tip_deflection_mm=fork_force*length**3/(3*E*Iy),
 restrained_rotation_tip_deflection_mm=fork_force*length**3/(12*E*Iy))
report=dict(scope='First-order screening only; solid homogeneous beam assumptions omit joints, holes, orthotropy and stress concentrations',
 material_reference='https://store.bblcdn.com/s7/default/b189de92249a4b9ebed28b8ea1f080f0/Bambu_PLA_Basic_Technical_Data_Sheet.pdf',
 assumed_modulus_MPa=E,reference_XY_bending_modulus_MPa=2750,
 torque_Nmm=T,ideal_worm_axial_force_N=worm_force,ideal_rail_force_N=rail_force,rail=rail,fork=fork,
 rail_guide_lip_slot=dict(upper_web_height_mm=3.1,span_mm=42.6,approx_local_deflection_at_1N_mm=42.6**3/(48*E*(5.4*3.1**3/12)),note='This local slot interrupts the full-depth section; the full-section estimate alone is not valid at the bolt load point'),
 qualified=False,limitations=['No strength claim for LEGO axles, teeth, friction pins or printed actuator teeth',
 'No printed-layer or infill model; material datasheet does not qualify this geometry',
 'A seized bolt can exceed the operating clearance budget even without breaking the rail',
 'Actual return-band load, friction and joint stiffness must be measured',
 'Fork free-rotation upper estimate plus joint play can exceed the original 1 mm positioning allowance'])
(R/'Load screening.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
