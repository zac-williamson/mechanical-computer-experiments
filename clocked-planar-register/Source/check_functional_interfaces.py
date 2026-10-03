"""Retained cartridge ports: centres, sleeve overlap and keyed phase/speed."""
from pathlib import Path
import json,hashlib,copy,math
import numpy as np
from wall_pose import joint
O=Path(__file__).resolve().parents[1]/'Functional register';P={p['id']:p for p in json.load(open(O/'parts.json'))}
f=dict(rail=0,rm=0,ro=0,rw=0,master_lift=4,slave_lift=4,angles={k:0. for k in ['D','WRITE','CLK','POWER','X','M','Q']})
for b in ['master','slave','write','clock']:f[b]=dict(q=0,g=0,b=0,w=0)
rows=[]
for sleeve,left,right in [('M interstage coupling','master output right 6L','slave data port axle'),('POWER coupling 49','master power port axle','POWER connection shaft'),('POWER coupling 147','POWER connection shaft','slave power port axle')]:
 ps=[P[n] for n in [left,sleeve,right]];b=[np.array(p['bounds']) for p in ps];centres=[v.mean(0) for v in b]
 overlap=[max(0.,min(b[i][1,0],b[1][1,0])-max(b[i][0,0],b[1][0,0])) for i in [0,2]]
 error=max(np.linalg.norm(c[1:]-centres[1][1:]) for c in centres)
 phase=[joint(p,f)[3] for p in ps];phase_error=max(abs((a-phase[1]+math.pi/4)%(math.pi/2)-math.pi/4) for a in phase)
 velocity_error=0.
 for d in f['angles']:
  ff=copy.deepcopy(f);ff['angles'][d]=1.;rates=[joint(p,ff)[3]-a for p,a in zip(ps,phase)];velocity_error=max(velocity_error,max(abs(a-rates[1]) for a in rates))
 gap=float(b[2][0,0]-b[0][1,0]);ok=error<1e-5 and phase_error<1e-6 and velocity_error<1e-8 and min(overlap)>=6.5 and 0<gap<2.1
 row=dict(coupling=sleeve,shaft_left=left,shaft_right=right,axis='X',centre_mm=centres[1].tolist(),sleeve_overlap_each_mm=overlap,shaft_end_gap_mm=gap,centre_error_mm=float(error),keyed_phase_error_deg=math.degrees(phase_error),max_velocity_error_rad=velocity_error,pass_check=bool(ok));rows.append(row);print(row,flush=True)
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),parts_sha256=hashlib.sha256((O/'parts.json').read_bytes()).hexdigest(),ports=rows,port_pass=all(e['pass_check'] for e in rows),assembly_sequence=['Keep both complete cartridges separate along X. Set their timing plates to matching positions.','Fit the M coupling and the two POWER couplings/bridge while moving the slave toward the master along -X. Internal bearings and shafts stay installed.','Seat both horizontal chassis pins, then press the front timing strap onto the four terminal pins already installed in the latch cartridges.','For separation, remove the timing strap and draw the complete slave along +X to disengage the couplings and frame pins.'],limits=['Nominal CAD centres, phase and engagement checks; not a tolerance stack, insertion-force or retention-force measurement.','WRITE/feedback and shared-controller connections remain unfinished.'])
(O/'Module interface checks.json').write_text(json.dumps(report,indent=2))
