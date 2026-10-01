from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'visible-carriage-candidate';T=R/'neck-candidate';B=T/'before-visible-carriage';B.mkdir(exist_ok=True)
for n in ['Rod tie functional checks.json','Complete actuator assembly checks.json','Rod tie final checks.json']:assert json.loads((O/n).read_text())['passed']
assert all(not p['islands'] for p in json.loads((O/'Rod tie print checks.json').read_text()))
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
for p in (O/'baseline').iterdir():
 if not (B/p.name).exists():shutil.copy2(p,B/p.name)
keep=['Model.json','Viewer.html','Print layout.stl','Carriage print layout.stl','Visible carriage print layout.stl','Bed orientation checks.json','Print layout checks.json','Clearance checks.json','Complete actuator assembly checks.json','Rod tie functional checks.json','Rod tie final checks.json','Rod tie print checks.json','Visibility changes.json','Visibility measurement.json','Visibility comparison.png','Review views.png']
for n in ['Carriage body','Carriage bearing end']:keep.extend([n+'.stl',n+' print.stl'])
for n in keep:shutil.copy2(O/n,T/n)
notes='''# Open-view carriage

The primary viewing direction is from -Y toward the XZ module plane. Visibility of the working mechanism is a design requirement: use open structural shapes rather than opaque covers where bearings and load paths allow it.

A single rounded sight opening replaces the central front panel: X=-9..9 mm, Z=11.3..28 mm, 2 mm corner radii. It removes carriage material only in Y=-1..14 mm. The two side bearing supports, the 2 mm lower fork-support beam, the reinforced upper bearing roots, keyed rod seats, rod-pin supports, locking pockets and lever-contact arm remain. No added envelope, new parts or relocated interfaces. The viewer opens in the -Y view.

The before/after comparison uses opaque meshes at identical pose and scale. It shows the worm and more of the reaction mechanism through the opening. Side bearings and the functional rod still obscure some areas; they are retained for their mechanical purpose.

Validation passed: 26 operating poses with printed, hardware and band envelopes; all previously checked straight assembly paths; unchanged clutch, lever, locking and worm-thrust surfaces; unchanged pin-support regions and key anti-rotation stops; two locking positions and 31 released-carriage positions; supported upward-facing worm bearing contact surfaces; new-material tile check (this revision is subtractive); single-solid watertight exports; no detached islands in a 0.2 mm layer screen; viewer control and axes checks, including narrow screens. The existing clutch-fork overhang remains and may require local slicer support. Physical stiffness, friction, fatigue and pin retention remain to be tested.

Use Visible carriage print layout.stl for both replacement carriage halves. Keep the keyed carriage rod and the two 2L rod-attachment pins from the preceding revision. Print layout.stl contains all current printed components. Older replacement packages contain superseded carriage geometry. The previous model is preserved in before-visible-carriage. Assembly instructions.md remains applicable.
'''
(T/'Visibility notes.md').write_text(notes)
(T/'README.md').write_text('# Open-view carriage\n\nCurrent design: [Visibility notes](Visibility%20notes.md). The central panel has been opened to show the mechanism from -Y into the XZ plane. The keyed rod connection and prior lock-guidance improvements remain.\n\nUse Visible carriage print layout.stl for the two carriage halves, or Print layout.stl for all current printed parts. Existing Assembly instructions.md still applies. Physical validation is pending; geometry and assembly checks are recorded in Visibility notes.md.\n')
(T/'Print notes.md').write_text('Use the current Visible carriage print layout.stl or Print layout.stl. Both carriage halves retain their outer-X print faces, vertical worm bores and upward-facing thrust surfaces. The old fork overhang is unchanged and may need local support. See Visibility notes.md for validation scope.\n')
h=json.loads((T/'Current geometry hashes.json').read_text())
for n in keep:h[n]=hashlib.sha256((T/n).read_bytes()).hexdigest()
(T/'Current geometry hashes.json').write_text(json.dumps(h,indent=2));print('Published open-view carriage',flush=True)
