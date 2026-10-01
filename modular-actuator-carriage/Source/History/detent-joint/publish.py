from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'detent-candidate';T=R/'neck-candidate';B=T/'before-detent-joint';B.mkdir(exist_ok=True)
checks=json.loads((O/'Clearance checks.json').read_text());assert all(not checks[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
assert json.loads((O/'Detent function checks.json').read_text())['passed']
for p in T.iterdir():
 if p.is_file() and not (B/p.name).exists():shutil.copy2(p,B/p.name)
s=(O/'Viewer.html').read_text().replace('<h1>Actuator, carriage and locking bolt</h1>','<h1>One-piece locking pockets</h1>').replace('Adapted from the planar register · two carriage pieces · four rows at 16 mm pitch.','Both locking pockets belong entirely to the bearing-end carriage piece. Explode the carriage to inspect the revised joint.').replace('<div class="tools"><label><input id="thin-strips"','<div class="tools" hidden><label><input id="thin-strips"').replace('id="thin-strips" type="checkbox" checked','id="thin-strips" type="checkbox"').replace("if(D.tiling)setmode('tiles');else draw();","setmode('assembly');")
s=s.replace('<a href="2x2%20frame','<a href="Print%20layout.stl">Updated print layout</a> · <a href="Carriage%20body%20print.stl">Carriage body STL</a> · <a href="Carriage%20bearing%20end%20print.stl">Bearing end STL</a> · <a href="2x2%20frame',1)
(O/'Viewer.html').write_text(s)
notes=(T/'Print notes.md').read_text()+'\n\n## Continuous locking pockets\nBoth pockets now belong wholly to the bearing-end carriage piece. A 0.2 mm clearance separates its keeper extension from the body. The bearing end now prints on its outer −X face, keeping its axle bores vertical and building the extension last. A solid backing below the body recess prevents a detached print island. Both changed pieces pass the 0.2 mm layer-connectivity screen. Use the new oriented STLs or full layout; previous carriage exports have the old joint. Geometric checks do not qualify physical strength.\n'
(O/'Print notes.md').write_text(notes)
keep=['Viewer.html','Model.json','Carriage body.stl','Carriage bearing end.stl','Carriage body print.stl','Carriage bearing end print.stl','Print layout.stl','Print layout checks.json','Print notes.md','Review views.png','Clearance checks.json','Carriage assembly access.json','Detent joint checks.json','Detent function checks.json','Detent print orientation checks.json','Bed orientation checks.json']
for n in keep:shutil.copy2(O/n,T/n)
(T/'Current geometry hashes.json').write_text(json.dumps({n:hashlib.sha256((T/n).read_bytes()).hexdigest() for n in keep},indent=2))
print('Published detent joint',flush=True)
