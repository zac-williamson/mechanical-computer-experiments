"""Rebuild the integrated-cam mechanical test revision and its finite checks."""
from pathlib import Path
import runpy,shutil
S=Path(__file__).resolve().parent;O=S.parents[1]/'work/integrated-cam-development';O.mkdir(exist_ok=True)
if (O/'Roller passage relief.json').exists():(O/'Roller passage relief.json').unlink()
for p in (S/'integrated-cam-docs').iterdir():shutil.copy2(p,O/p.name)
for name in ['integrated_cam.py','redesign_bearing_frame.py','detachable_bolt_guide.py','integrated_timing.py','check_integrated_loading.py','check_integrated_travel.py','check_integrated_transition_geometry.py','check_integrated_hardware.py','check_integrated_rotation.py','check_integrated_actuator.py','check_integrated_crossbank.py','prepare_integrated_prints.py','check_support_free_prints.py','check_detachable_guide.py','check_bolt_y_restraint.py','prepare_lock_fit_test.py','fix_clutch_axles.py','align_keyed_hardware.py','phase_connected_gears.py','coupled_register.py','publish_coupled_viewer.py','check_bearing_frame.py','check_restored_bearings.py','check_frame_printed_clearance.py','check_frame_native_contacts.py','check_axle_stops.py','check_coupled_model.py','check_coupled_assembly.py','package_coupled_revision.py']:
 if name=='detachable_bolt_guide.py':shutil.copy2(O/'Unified rear backbone.stl',O/'Frame before detachable guide.stl')
 print('Running',name,flush=True);runpy.run_path(str(S/name),run_name='__main__')
