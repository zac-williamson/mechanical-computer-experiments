from pathlib import Path
import shutil
R=Path(__file__).resolve().parents[1];O=R.parent/'work/integrated-cam-development'
shutil.copy2(R.parent/'work/register-before-left-cam/Clutch phase contact scan.json',O/'Clutch phase contact scan.json')
s=(R/'Source/audit_transition_timing.py').read_text().replace('from direct_cam_math import cam_lift','from integrated_cam_math import cam_lift').replace("O=R/'Planar register'", "O=R.parent/'work/integrated-cam-development'").replace('5.4','3.2').replace('abs(q)>=3.3','abs(q)>=3.0').replace('nominal_bolt_first_intrusion_qe_mm=-.75','nominal_bolt_first_intrusion_qe_mm=-1.0833333333333333')
exec(compile(s,str(R/'Source/audit_transition_timing.py'),'exec'))
