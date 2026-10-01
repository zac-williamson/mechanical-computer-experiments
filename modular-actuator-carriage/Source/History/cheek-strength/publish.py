from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'strength-candidate';T=R/'neck-candidate';B=T/'before-cheek-strength';B.mkdir(exist_ok=True)
for p in T.iterdir():
 if p.is_file() and not (B/p.name).exists():shutil.copy2(p,B/p.name)
p=O/'Viewer.html';s=p.read_text().replace('<h1>Actuator, carriage and locking bolt</h1>','<h1>Reinforced actuator cheeks</h1>').replace('Adapted from the planar register · two carriage pieces · four rows at 16 mm pitch.','Cheek mounting necks widened from 0.2 to 5.2 mm · bolt-guide walls reinforced · accepted carriage changes retained.').replace('<div class="tools"><label><input id="thin-strips"','<div class="tools" hidden><label><input id="thin-strips"').replace('id="thin-strips" type="checkbox" checked','id="thin-strips" type="checkbox"').replace("if(D.tiling)setmode('tiles');else draw();","setmode('assembly');")
s=s.replace('<a href="2x2%20frame','<a href="Print%20layout.stl">Updated full print layout</a> · <a href="Upper%20actuator%20cheek%20print.stl">Upper cheek STL</a> · <a href="Lower%20actuator%20cheek%20print.stl">Lower cheek STL</a> · <a href="2x2%20frame',1)
p.write_text(s)
(O/'Print notes.md').write_text('''# Current print notes

Both actuator-cheek mounting necks were 0.2 mm wide at Y=26.5; they are now 5.2 mm wide through their existing 4 mm plate thickness. Print on the lower Z face: working axle bores remain vertical. The upper cheek joining-hole roofs have been opened into adjoining cavities to remove thin shelves; fitted pin endpoints remain unchanged. The guide front side walls are reinforced to 2 mm in X over Y20–25, Z40.5–47.2, and its rear floor edge is braced. Its band insertion check passes.

The full layout contains 12 watertight solids on a 202.7 × 233.9 mm footprint. No detached islands were found in the 0.2 mm layer screen. These checks do not establish physical strength or support-free printing. The guide prints rear +Y face down. Use the separately named print.stl exports for the changed parts.

Remaining risks: thin carriage clearance edges, short unsupported bearing-thrust ledges, 0.8 mm band-retaining lips, and rod-hole ligaments still require physical testing. The scan found no comparable 0.2 mm mounting neck in the lever, bearing walls or control rods. Local support is still required beneath the raised rod lap ends. Keep support off running surfaces. All checks are geometry screens, not load/fatigue tests or slicer simulations.
''')
(O/'README.md').write_text('''# Reinforced actuator cheeks

Current geometry includes the accepted carriage-root reinforcement and flat fork face. Upper and lower cheek mounting webs are broadened; upper joining-hole shelves are opened; the bolt-guide front walls and floor backing are reinforced. No axle, pin or module-interface positions changed.

See Print notes.md for remaining print risks. Clearance checks cover 26 operating positions; band insertion checks cover 68 positions; all-part layer and planar-thickness screens are supplied. These do not guarantee physical reliability. Saved printing snapshots are untouched.

Regenerate using cheek-strength/build.py, cheek-strength/finish.py under the shared throttled runner, then cheek-strength/publish.py.
''')
keep=['Viewer.html','Model.json','Print layout.stl','Print layout checks.json','Print notes.md','README.md','Review views.png','Clearance checks.json','All-part print screen.json','Neck measurements.json','Cheek reinforcement.json','Band insertion checks.json','Bed orientation checks.json']
keep += [p.name for p in O.glob('*.stl') if 'fit sample' not in p.name and p.name!='Fit test layout.stl']
for n in set(keep):shutil.copy2(O/n,T/n)
(T/'Current geometry hashes.json').write_text(json.dumps({n:hashlib.sha256((T/n).read_bytes()).hexdigest() for n in sorted(set(keep))},indent=2))
