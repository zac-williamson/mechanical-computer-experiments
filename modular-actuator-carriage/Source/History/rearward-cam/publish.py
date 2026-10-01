from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[1];O=R/'rearward-cam-candidate';T=R/'neck-candidate';B=T/'before-rearward-cam'
assert (T/'Model.json').read_bytes()==(O/'baseline/Model.json').read_bytes()
assert json.loads((O/'Cam extension checks.json').read_text())['passed']
assert all(not p['islands'] for p in json.loads((O/'Cam print checks.json').read_text()))
assert json.loads((O/'Print layout checks.json').read_text())['watertight']
notes='''# Cam support moved toward the bolt

The central curved portion of the orange Lock control rod extends in +Y from Y14.2 to Y19.6, over local X-12.5 to X+12.5. The original XZ cam section is copied, so its lift and timing are retained. The extension continues from the same flat Z40 print face, rather than forming an unsupported ledge.

Move the existing front follower bush +4 mm along Y, from Y11.6–15.6 to Y15.6–19.6 (centre Y13.6 to Y17.6). The same 3L follower axle and rear bush are retained. The guide starts at Y20, leaving 0.4 mm nominal clearance to the extended cam. The guide itself is unchanged.

Simply widening the cam without moving the bush would not relocate the lifting force. Moving both reduces the nominal lever arm relative to the guided-head centre Y27.6 from 14 to 10 mm. That is a centre-based estimate of about 29% less pitching moment for the same force, not a physical measurement. Actual load distribution across the contact width, friction, clearances and elastic forces affect the result.

## Printing and assembly

Reprint only Lock control rod, using Lock control rod print.stl or Lock rod print layout.stl. Keep the supplied Z40 side on the bed. Its pin-hole roof bridges and end couplings are unchanged. Slide the front follower bush +4 mm along the existing axle before fitting the rod. Do not shift the rear bush or change the axle length. The 9L actuator axle, 4L output axles, smooth module connectors and single locking band remain as in the previous revision.

Use Print layout.stl for a complete current set. Older rod replacement layouts are historical and do not include this extension.

## Checks

- 26 full operating poses: no new non-mating printed/hardware/band interference.
- Additional 39 release positions: the added material clears non-mating components. Intended follower/cam contact matches the inherited profile per unit contact width; its existing polygonal contact approximation was not treated as a new clash.
- Original rod material and central cam section preserved to numerical tolerance. Rod length, pin locations, X stops and mounting rows retained.
- Follower remains within the original axle length.
- Printed rod is one watertight solid; the re-exported full layout is watertight. Layer-connectivity checks at 0.2 mm spacing find no detached islands. The new extension starts at the bed; these checks do not certify every inherited bridge on the rod for every printer.
- Viewer retains fixed labelled axes and all previous module-connection views. Inspect 'Lock rod, follower and bolt' for this change.

Mechanical stiffness, closing reliability and actual friction still require a printed test.
'''
(O/'Cam extension notes.md').write_text(notes)
p=O/'Assembly instructions.md';p.write_text(p.read_text()+'\n## Rearward cam support\n\nUse the updated locking rod. Move the front follower bush +4 mm along Y, to Y15.6–19.6; keep the rear bush and 3L follower axle unchanged. See Cam extension notes.md.\n')
(O/'README.md').write_text('# Current module\n\nLatest change: orange locking rod cam extends to Y19.6 and its follower bush moves +4 mm along Y, reducing the nominal pitching lever arm. See Cam extension notes.md.\n\nOnly the locking rod needs reprinting. Use Lock control rod print.stl, Lock rod print layout.stl, or the updated full Print layout.stl. Whole-stud axles, smooth module connectors, the reinforced single-band guide and braced fork are retained.\n')
(O/'Print notes.md').write_text('Latest changed part: Lock control rod. Print Lock control rod print.stl in the supplied orientation, with Z40 on the bed. The cam extension starts on this same plane. Full Print layout.stl is updated. Older rod replacement layouts are historical. Inherited carriage print caveats remain: worm bearing faces upward; the existing fork contact lip may need local support. See Cam extension notes.md.\n')
B.mkdir(exist_ok=True)
names=['Lock control rod.stl','Lock control rod print.stl','Lock rod print layout.stl','Print layout.stl','Model.json','Viewer.html','Bed orientation checks.json','Print layout checks.json','Clearance checks.json','Cam extension changes.json','Cam extension checks.json','Cam print checks.json','Cam extension notes.md','Cam extension views.png','README.md','Print notes.md','Assembly instructions.md']
for n in names:
 if (T/n).exists() and not (B/n).exists():shutil.copy2(T/n,B/n)
 shutil.copy2(O/n,T/n)
p=T/'Current geometry hashes.json';h=json.loads(p.read_text())
for n in names:h[n]=hashlib.sha256((T/n).read_bytes()).hexdigest()
p.write_text(json.dumps(h,indent=2));print('Published cam extension, follower position and updated print files.')
