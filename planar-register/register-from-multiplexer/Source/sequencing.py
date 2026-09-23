"""Kinematic design for a 5:1 rocking lever driven by the write carriage's cam.
This sizes the linkage; it is NOT an integrated/force-qualified cam assembly.
"""
from pathlib import Path
import json,math,numpy as np
from shapely.geometry import LineString
R=Path(__file__).resolve().parents[1];O=R/'Investigation'
# Full directional backlash, not the viewer's sign(q) shortcut.
fork_half_play=.4;left_gear_inner=-12.;ring_left=-11.2
last_possible_D_contact=left_gear_inner-ring_left+fork_half_play # -0.4
# Allow an additional 0.2mm axial stack uncertainty.
dangerous_q=last_possible_D_contact+.2
# 50mm output radius, 10mm follower radius; both on same side of pivot.
# Extra open stroke provides allowance for cam/guide clearance amplification.
def command(q):return -15.+20.*float(np.clip((q-.9)/1.5,0,1))
def centre(q):
 u=(5.8+command(q))/50
 return (50-10*math.sqrt(1-u*u)+q,6.7+10*u)
path=LineString([centre(q) for q in np.linspace(-8,8,3201)])
slot=path.buffer(3.8,quad_segs=64)
# Geometric allowable roller centres; the 3.6mm roller is not a point follower.
free=slot.buffer(-3.6,quad_segs=64)
# Sample rocker angle plus longitudinal guide/manufacturing offset. Convert angle
# directly back to bolt command. Checks include corners of the finite-radius track.
from shapely import points, covers
angles=np.radians(np.linspace(-20,20,16001));xs=50-10*np.cos(angles);zs=6.7+10*np.sin(angles);commands=50*np.sin(angles)-5.8
results=[]
for q in np.linspace(-4.6,4.6,185):
 allowed=[]
 for dx in [-.6,0,.6]:
  mask=covers(free,points(xs+float(q)+dx,zs));allowed.extend(commands[mask].tolist())
 results.append(dict(q=float(q),command_min_mm=min(allowed) if allowed else None,command_max_mm=max(allowed) if allowed else None))
unsafe=[x for x in results if x['q']<=dangerous_q and (x['command_max_mm'] is None or x['command_max_mm']> -9.6)]
hold=[x for x in results if 3.35<=x['q']<=4.6]
report=dict(status='CAM/ROCKER KINEMATIC STUDY — installation not yet generated',D_last_possible_contact_q_mm=last_possible_D_contact,D_contact_with_stack_allowance_q_mm=dangerous_q,cam_start_q_mm=.9,cam_end_q_mm=2.4,output_radius_mm=50,follower_radius_mm=10,cam_roller_radius_mm=3.6,slot_radius_mm=3.8,longitudinal_error_test_mm=[-.6,.6],required_command_for_0_4mm_bolt_clearance=-9.6,unsafe_reconnection_cases=unsafe,minimum_hold_command_mm=min(x['command_min_mm'] for x in hold if x['command_min_mm'] is not None),linear_cam_rejected=dict(required_slope=15.6/2.4,guide_friction_self_wedging_threshold=2.4/15.6,reason='A direct sliding follower at this slope can wedge. A rocking lever transfers the lateral reaction to an axle bearing.'),notes=['Rocker output presses the bolt downward to unlock. An elastic band pulls the bolt toward lock, with a positive stop at full insertion.','The band must accommodate at least 11.2mm additional lost travel when insertion is obstructed. Select by measured force-extension curve, target no more than 3N.','Rocker pivot, roller retention, band route, brackets and cam-to-write-carriage attachment are not yet validated in the assembly.','A command clearance proof is not a proof of actual withdrawal if the linkage bends, binds or breaks.'],samples=results)
(O/'Sequencing study.json').write_text(json.dumps(report,indent=2));(O/'Cam slot.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg" viewBox="20 -15 40 20">'+slot.svg(fill_color='#d9a33e')+'</svg>')
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
