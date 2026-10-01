from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1];O=R/'simplified-carriage-r1'
p=O/'Viewer.html';s=p.read_text().replace('<h1>Actuator, carriage and locking bolt</h1>','<h1>Carriage simplification — revision 1</h1>').replace('Adapted from the planar register · two carriage pieces · four rows at 16 mm pitch.','Separate revision: fork support simplified; thin remnants removed; lever-side recess backed. Saved print model unchanged.').replace('<div class="tools"><label><input id="thin-strips"','<div class="tools" hidden><label><input id="thin-strips"').replace('if(D.tiling)setmode(\'tiles\');else draw();',"setmode('assembly');")
s=s.replace('<a href="2x2%20frame', '<a href="Print%20layout.stl">Revision 1 print layout STL</a> · <a href="../adapted/Viewer.html">Original print model</a> · <a href="2x2%20frame',1)
p.write_text(s)
keep={'Recess gap checks.json','Model.json','Development checks.json','Tiling geometry.json','Source provenance.json','Simplification changes.json','Carriage interface checks.json','Carriage strength probes.json','Carriage assembly access.json','Split assembly insertion checks.json','Clearance checks.json','Bed orientation checks.json','Revision print checks.json','Print layout checks.json','Bearing print test.json'}
h=O/'baseline-reference';h.mkdir(exist_ok=True)
for p in list(O.glob('*.json')):
 if p.name not in keep:p.replace(h/p.name)
for name in ['Fit test layout.stl','Rod guide fit sample.stl','Rod fit sample.stl','Lock pocket fit sample.stl','Fixed axle bearing fit sample.stl']:
 p=O/name
 if p.exists():p.replace(h/name)
(O/'README.md').write_text('''# Carriage simplification — revision 1

Separate review revision; the adapted viewer and saved printing snapshot are unchanged.

The existing two-piece topology is retained. Rectangular trimming removes the two 0.2 mm worm-bearing fringes, opens the thin stepped pivot-clearance web, and removes the small reaction-clearance ledge. The unused recess in the far −X end of the lever-side support is backed with continuous material at X −16.25 to −8.1, Y 35.2 to 40.4, Z 11.49 to 20.41 mm (overlapping both recess boundaries to eliminate the previous 0.1 and 0.4 mm gaps). The active lever-contact profile is preserved. The layered central clutch-fork mounting region is replaced by a continuous saddle between X −8 and 7.8 mm, with cylindrical clutch and worm clearance. The original fork contact material and its connecting root are retained; the support is not flattened through the rotating clutch envelope.

This is a focused cleanup, not a complete replacement of all legacy carriage surfaces. Exact added and removed volumes are recorded in Simplification changes.json. It is not a weight-reduction or measured-friction improvement.

Validation: both halves remain single watertight solids; existing working-interface and structural-section probes pass; 26 mechanism poses show no unintended printed, hardware or band collisions; carriage/pin/rod insertion checks pass. The two oriented carriage meshes have no detached islands in a 0.2 mm layer screen. These are geometric checks, not physical strength, torque or fatigue qualification.

Use Viewer.html for the revision and Print layout.stl for the matching 12-part layout. Axes and independent rotation controls are retained. Rebuild with simplification/build.py then simplification/finish.py, serially through the shared throttled runner, then simplification/publish_review.py.

Historical copied reports and old fit coupons are under baseline-reference and do not qualify this revision. Other module geometry is unchanged. Carriage bores print vertically, but existing thrust-shoulder ledges still require a sample print; rod lap ends require local support. Physical assembly and loaded cycling remain necessary.
''')
(O/'Print notes.md').write_text('''# Revision 1 print notes

Print layout.stl contains all twelve module parts in the audited orientations (202.7 × 233.9 mm footprint). Only the two carriage parts changed. Both print on their joint faces, with axle bores vertical. The removed thin remnants no longer need to print; the lever-side recess is backed at its unused −X end.

The carriage thrust-shoulder ledges still require inspection on a physical print. Bearing print test.stl is regenerated from this revision. No detached islands were found in the 0.2 mm carriage layer screen. This does not certify support-free printing. Other parts retain their prior support requirements, particularly local support beneath the raised rod lap ends. Never place support on axle bores or running contact surfaces.

Test fit, free movement, lock release, both clutch endpoints and loaded cycling before treating this as qualified. Printer/material settings remain unspecified.
''')
(O/'Revision manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.iterdir() if p.is_file() and p.suffix in ['.stl','.html']},indent=2))
