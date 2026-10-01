from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'lock-guidance-candidate';T=R/'neck-candidate';B=T/'before-lock-guidance';B.mkdir(exist_ok=True)
for p in (O/'baseline').iterdir():
 if not (B/p.name).exists():shutil.copy2(p,B/p.name)
for n in ['Lock improvement checks.json','Tilted entry checks.json','Band insertion checks.json']:assert json.loads((O/n).read_text())['passed']
assert all(not r['islands'] for r in json.loads((O/'Lock print checks.json').read_text()))
keep=['Model.json','Viewer.html','Print layout.stl','Carriage print layout.stl','Lock improvement print layout.stl','Bed orientation checks.json','Print layout checks.json','Review views.png','Clearance checks.json','Lock improvement checks.json','Tilted entry checks.json','Band insertion checks.json','Lock print checks.json']
for n in ['Locking bolt guide','Locking bolt','Carriage bearing end']:keep += [n+'.stl',n+' print.stl']
html=(O/'Viewer.html').read_text().replace('Open carriage — actuator assembly access','Improved locking bolt guidance').replace('Rear frame removed from the right half · open interior · joining pins at the front. Use “Assembly sequence” to inspect fitting around the built actuator.','Longer guided bolt shoulder · flared pocket entrances · three matching replacement parts.').replace('<a href="Carriage%20print%20layout.stl">','<a href="Lock%20improvement%20print%20layout.stl">Lock replacement set STL</a> · <a href="Lock%20revision%20notes.md">Lock revision notes</a> · <a href="Carriage%20print%20layout.stl">',1)
(O/'Viewer.html').write_text(html)
for n in keep:shutil.copy2(O/n,T/n)
text='''# Lock guidance and entry revision

Fit these three replacement parts together: locking bolt, locking bolt guide, and carriage bearing end. Lock improvement print layout.stl contains this set; Print layout.stl contains the full current module.

The bolt has a longer lower rear guided shoulder with a sloping transition for printing. The guide has matching clearance and longer corner contact surfaces. Its X clearance is 0.25 mm per side at the new contact surfaces; Y clearance is 0.2 mm per side there. The lower front shoulders also provide a seating stop when closed. This constrains the bolt itself, while the carriage pockets have more generous entry clearance. The guide keeps the same overall envelope. Previously removed thin bridges remain absent.

The two pocket mouths flare over their upper 0.7 mm; the bolt nose tapers over its lower 0.8 mm. Straight X pocket clearance is 0.35 mm per side. There is 1.5 mm of straight shank-to-pocket overlap at full engagement. Both pockets remain in one carriage piece. Rods, cam profile, bands, axles, module grid and external interfaces are unchanged.

Print the guide on its -Z bottom, as supplied. Its fixed band-peg flange lower tips are flattened to start with the shafts; upper retaining lips remain. The moving bolt retains its front-face print orientation. The carriage end retains its outer -X print face, with the worm-bearing contact face upward. No detached islands were found in the 0.2 mm layer screen; this is not a complete slicer support audit.

Checks: 26 operating poses without reported printed/hardware/band interference; 39 bolt lift positions; 31 released carriage positions; straight detent stops; initial nose entry at +/-0.8 mm separately in X or Y; 32 initial-entry combinations of +/-1.5 degree tilt with +/-0.3 mm X and +/-0.2 mm Y displacement; accessible band installation. Angular figures are prescribed-pivot geometric checks, not a bound on all possible rigid-body motion. Spring-driven self-centering, friction and loaded locking still require a physical test.

Use current Lock improvement checks.json, Tilted entry checks.json, Band insertion checks.json and Lock print checks.json for this revision. Earlier lock/guide reports describe superseded parts. The previous print model is saved in before-lock-guidance.
'''
(T/'Lock revision notes.md').write_text(text)
p=T/'README.md';p.write_text('Current locking revision: see [Lock revision notes](Lock%20revision%20notes.md). Use the three-part Lock improvement print layout.stl. Earlier locking geometry checks below are superseded.\n\n'+p.read_text())
h=json.loads((T/'Current geometry hashes.json').read_text())
for n in keep:h[n]=hashlib.sha256((T/n).read_bytes()).hexdigest()
(T/'Current geometry hashes.json').write_text(json.dumps(h,indent=2));print('Published lock improvements',flush=True)
