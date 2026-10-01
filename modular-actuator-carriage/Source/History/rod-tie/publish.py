from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'rod-tie-candidate';T=R/'neck-candidate';B=T/'before-rod-tie';B.mkdir(exist_ok=True)
for n in ['Rod tie functional checks.json','Complete actuator assembly checks.json','Rod tie final checks.json']:assert json.loads((O/n).read_text())['passed']
assert all(not p['islands'] for p in json.loads((O/'Rod tie print checks.json').read_text()))
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
for p in (O/'baseline').iterdir():
 if not (B/p.name).exists():shutil.copy2(p,B/p.name)
keep=['Model.json','Viewer.html','Print layout.stl','Carriage print layout.stl','Rod tie print layout.stl','Bed orientation checks.json','Print layout checks.json','Clearance checks.json','Complete actuator assembly checks.json','Rod tie functional checks.json','Rod tie final checks.json','Rod tie print checks.json','Rod tie changes.json','Review views.png','Rod connection views.png']
for n in ['Carriage body','Carriage bearing end','Carriage control rod']:keep.extend([n+'.stl',n+' print.stl'])
for n in keep:shutil.copy2(O/n,T/n)
notes='''# Compact carriage: keyed rod connection

Replace both carriage halves and the carriage control rod together, using Rod tie print layout.stl. Keep the existing two 2L rod-attachment pins. Omit the two X-axis carriage joining pins; their projecting mounts are removed. The rod now connects both halves through integral locating keys and the two pins. Three-length pins are not used because the inward extension of the right pin intersects the actuator pivot axle.

The carriage front limit returns from Y=-6.4 to Y=1.55, a 7.95 mm reduction. External axle/rod coordinates, rod length, end coupling holes, X/Z module pitch, cam, bands, locking guide and locking bolt remain unchanged. The improved locking pockets remain in one carriage half.

The rod carries two 3.5 mm wide keys at X=-6.25..-2.75 and X=2.75..6.25. They project toward +Y to Y=16.2. Their lower faces ramp upward at 45 degrees, starting above the reaction-axle hardware. Matching carriage seats have 0.15 mm clearance at their outer X sides and upper Z faces, with clearance beneath the ramp. Seats open into the carriage split, avoiding a thin inner wall. Rod-pin support material deeper than Y=16.5 is unchanged. The full pin bore remains separated from the seat by at least 2.1 mm in X. The rod itself is not weakened by new recesses.

Validation: 26 operating poses checked against printed parts, native hardware envelopes and bands; straight assembly approaches sampled every 0.5 mm; both locking states and 31 released-carriage positions; unchanged clutch, lever, detent and worm thrust regions; continuous support beneath the worm bearing contact faces; both keys contact their seats under rotation in either direction (1.25–1.75 degrees in the prescribed-pivot clearance test); new material clear of adjacent tiles; all changed meshes single watertight solids; no detached islands in a 0.2 mm print-layer screen. The assembled parts still depend on pin retention and rod stiffness: no force, fatigue, friction or physical print test has been performed.

Print both carriage halves on their existing outer X faces; worm bores remain vertical and thrust faces face upward with solid material beneath. Print the rod on its -Z side as supplied. The new key ramps grow from the rod without starting unsupported islands. This change does not alter the existing clutch-fork overhang; the layer-connectivity test is not a complete slicer support audit for every pre-existing feature. Inspect that unchanged area when slicing.

For review, select Carriage and keyed rod in the viewer and use Explode or Assembly sequence. Prior print files are preserved in before-rod-tie. The earlier Lock improvement print layout.stl is a previous revision and should not be used for the current carriage end. Use Rod tie print layout.stl or Print layout.stl.
'''
(T/'Rod tie notes.md').write_text(notes)
(T/'Assembly instructions.md').write_text('''# Assemble the compact carriage

1. Assemble the reaction gear, lever and both actuator cheeks, including their axles, retaining bushes and joining pins. Hold the lever near neutral. Leave the worm input axle out.
2. Hold the worm in position and bring the main/right carriage half from -Y.
3. Bring the bearing-end/left half from -X. Support both halves while fitting the rod; the removed joining pins no longer hold them together during this step.
4. Preload each existing 2L rod-attachment pin into its carriage half from -Y, leaving the rod-facing ends exposed. Do not try to push a pin's centre collar through the assembled rod.
5. Bring the keyed carriage rod from -Y onto both exposed pins. The keys enter their seats and locate the two halves.
6. Insert the worm input axle along X, then fit the remaining locking mechanism, bearing walls and retainers.

The viewer's assembly sequence shows steps 2–5. These approach paths were checked with the complete actuator in place at its neutral assembly position. Physical ease of handling and pin insertion force require a bench test.
''')
(T/'README.md').write_text('# Compact carriage with keyed rod\n\nThe current revision removes the raised carriage joining-pin mounts. Read [Rod tie notes](Rod%20tie%20notes.md) and [Assembly instructions](Assembly%20instructions.md). Use Rod tie print layout.stl for both carriage halves and the revised rod, or Print layout.stl for all current printed components.\n\nThe previous lock-guidance improvements are retained. Geometry checks pass; physical strength, friction and pin retention remain untested. Historical reports not listed in Rod tie notes describe earlier revisions.\n')
(T/'Print notes.md').write_text('Current orientations and limitations are in Rod tie notes.md. The full module layout is approximately 208.4 by 161.8 mm, with maximum print height 42.85 mm. The replacement set contains both carriage halves and the keyed carriage rod. Older locking-only replacement layouts contain a superseded carriage half.\n')
h=json.loads((T/'Current geometry hashes.json').read_text())
for n in keep:h[n]=hashlib.sha256((T/n).read_bytes()).hexdigest()
(T/'Current geometry hashes.json').write_text(json.dumps(h,indent=2));print('Published compact keyed carriage',flush=True)
