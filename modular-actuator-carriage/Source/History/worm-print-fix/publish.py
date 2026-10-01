from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'worm-candidate';T=R/'neck-candidate';B=T/'before-worm-print-fix';B.mkdir(exist_ok=True)
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
assert json.loads((O/'Detent function checks.json').read_text())['passed']
assert all(not r['detached_islands'] and r['bearing_contact_support_void_mm3']<.002 for r in json.loads((O/'Worm bearing print checks.json').read_text()))
for p in T.iterdir():
 if p.is_file() and not (B/p.name).exists():shutil.copy2(p,B/p.name)
s=(O/'Viewer.html').read_text().replace('<h1>Actuator, carriage and locking bolt</h1>','<h1>Carriage: clear worm and printable bearings</h1>').replace('Adapted from the planar register · two carriage pieces · four rows at 16 mm pitch.','Open worm clearance · upward-facing bearing contact surfaces · two carriage pieces. Both locking pockets remain in one piece.').replace('<div class="tools"><label><input id="thin-strips"','<div class="tools" hidden><label><input id="thin-strips"').replace('id="thin-strips" type="checkbox" checked','id="thin-strips" type="checkbox"').replace("if(D.tiling)setmode('tiles');else draw();","setmode('assembly');")
s=s.replace('<a href="2x2%20frame','<a href="Carriage%20print%20layout.stl">Updated carriage print layout</a> · <a href="Carriage%20print%20orientations.png">Bearing print orientations</a> · <a href="Print%20layout.stl">Full print layout</a> · <a href="2x2%20frame',1)
s=s.replace('The carriage split places the clutch-contact face on the bed. Test the bearing-shoulder bridges on the supplied small sample before printing a full carriage.','The carriage body prints on its outer +X face; the bearing-end piece prints on its outer −X face. Both worm contact faces point upward, with vertical bores and solid material beneath them. Supports elsewhere must stay off working surfaces.')
# Add a focused inspection selection without changing normal model controls.
s=s.replace("'<option value=\"all\">Whole module</option>';for(const p of scene())", "'<option value=\"all\">Whole module</option><option value=\"worm-group\">Carriage and worm</option>';for(const p of scene())")
s=s.replace("if($('part').value!=='all'&&p.name!==$('part').value)continue;", "if($('part').value==='worm-group'){if(!['Carriage body','Carriage bearing end','U015'].includes(p.name))continue;}else if($('part').value!=='all'&&p.name!==$('part').value)continue;")
(O/'Viewer.html').write_text(s)
(O/'Print notes.md').write_text('''# Current carriage print orientation

Print Carriage body with its outer +X face on the bed. Print Carriage bearing end with its outer −X face on the bed. Use Carriage print layout.stl or the individually named print.stl files: their orientations are already set. The worm axle bores are vertical. The worm-facing bearing surfaces are upward-facing, 7.15 mm above the bed, and have continuous solid carriage material beneath their contact annuli. No support is required beneath these bearing surfaces. Do not use the old bearing-test coupon or older carriage layouts for this revision.

The original lever-contact arm now belongs to the bearing-end half, so it no longer projects beyond the other half's +X printing face. The original lever profile and both unsplit locking pockets are retained. A straight 0.2 mm clearance joint permits assembly along X. The body retains a continuous 3.1 mm rear wall behind this arm.

The worm tunnel has been opened between the original end-bearing faces. The checked full-rotation envelope has at least 0.98 mm radial clearance from carriage material throughout the available axial float. Only the worm end faces are intended to contact the carriage. The existing axial spacing has not been changed.

Both carriage parts are watertight single solids and have no detached islands in the 0.2 mm layer screen. Those checks do not establish that all other faces print support-free: the clutch fork, transverse pin holes and other overhangs still need slicer inspection. Keep support away from contact faces. Earlier actuator-cheek reinforcements and bolt-guide reinforcements remain unchanged; other print risks from that review remain physical test items. Geometric validation does not establish friction, fatigue life or loaded operation.
''')
(O/'README.md').write_text('''# Worm clearance and bearing print correction

Current revision preserves the strengthened actuator cheeks, reinforced carriage bearing roots, flat fork face, unsplit locking pockets, original lever contact geometry and external module interfaces.

The lever-contact arm is now integral with the bearing-end half. This frees the body to print on +X, while the end prints on −X. Both bores are vertical and both worm-contact thrust faces print upward from solid underlying material. The close-fitting worm tunnel is opened to about 1 mm minimum radial clearance.

Inspect Carriage and worm in the viewer; use Explode to inspect ownership and the straight sliding joint. Bed orientations shows the exported layout. Carriage print layout.stl contains just the two revised parts; Print layout.stl contains all twelve printed module parts.

Current checks: Worm bearing print checks.json, Worm clearance checks.json, Clearance checks.json, Carriage assembly access.json, Detent function checks.json and Viewer checks.json. Earlier all-part and detent-only orientation reports predate this carriage revision. See Print notes.md for limitations and remaining print risks. Original printing snapshots are unchanged; before-worm-print-fix preserves the previous viewer and meshes.

Sources: worm-print-fix/build.py, finish.py, print_view.py and publish.py. Run geometry scripts with work/run_queued_cool_job.py to preserve the shared serial CPU throttle.
''')
keep=['Viewer.html','Model.json','Carriage body.stl','Carriage bearing end.stl','Carriage body print.stl','Carriage bearing end print.stl','Carriage print layout.stl','Carriage print layout checks.json','Carriage print orientations.png','Print layout.stl','Print layout checks.json','Print notes.md','README.md','Review views.png','Clearance checks.json','Carriage assembly access.json','Detent function checks.json','Worm print fix.json','Worm bearing print checks.json','Worm clearance checks.json','Bed orientation checks.json']
for n in keep:shutil.copy2(O/n,T/n)
(T/'Current geometry hashes.json').write_text(json.dumps({n:hashlib.sha256((T/n).read_bytes()).hexdigest() for n in keep},indent=2))
print('Published worm print correction',flush=True)
