from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'rod-candidate';T=R/'neck-candidate';B=T/'before-rod-side-print';B.mkdir(exist_ok=True)
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
assert json.loads((O/'Carriage assembly access.json').read_text())['passed']
c=json.loads((O/'Rod side-print checks.json').read_text());assert c['cam_surface_change_mm3']<.002 and all(not r['detached_islands'] for r in c['parts'])
for p in T.iterdir():
 if p.is_file() and not (B/p.name).exists():shutil.copy2(p,B/p.name)
s=(O/'Viewer.html').read_text().replace('<h1>Actuator, carriage and locking bolt</h1>','<h1>Rods with flat side-print faces</h1>').replace('Adapted from the planar register · two carriage pieces · four rows at 16 mm pitch.','Both rod ends print directly on the bed · locking cam unchanged · revised locking-rod guides.').replace('<div class="tools"><label><input id="thin-strips"','<div class="tools" hidden><label><input id="thin-strips"').replace('id="thin-strips" type="checkbox" checked','id="thin-strips" type="checkbox"').replace("if(D.tiling)setmode('tiles');else draw();","setmode('assembly');")
s=s.replace('<a href="2x2%20frame','<a href="Rods%20and%20guides%20print%20layout.stl">Updated rods and guides STL</a> · <a href="Rod%20print%20orientations.png">Rod print orientations</a> · <a href="Print%20layout.stl">Full print layout</a> · <a href="2x2%20frame',1)
s=s.replace('The carriage split places the clutch-contact face on the bed. Test the bearing-shoulder bridges on the supplied small sample before printing a full carriage.','Rods print on their flat −Z sides. Use the revised bearing walls with the locking rod. Both carriage worm bearings retain vertical bores and upward-facing contact faces. Keep support off working surfaces.')
s=s.replace("'<option value=\"all\">Whole module</option>';for(const p of scene())", "'<option value=\"all\">Whole module</option><option value=\"worm-group\">Carriage and worm</option><option value=\"rod-group\">Both rods</option><option value=\"guide-group\">Rods and bearing walls</option>';for(const p of scene())")
s=s.replace("if($('part').value!=='all'&&p.name!==$('part').value)continue;", "const groups={'worm-group':['Carriage body','Carriage bearing end','U015'],'rod-group':['Carriage control rod','Lock control rod'],'guide-group':['Carriage control rod','Lock control rod','Left bearing wall','Right bearing wall']};if(groups[$('part').value]){if(!groups[$('part').value].includes(p.name))continue;}else if($('part').value!=='all'&&p.name!==$('part').value)continue;")
(O/'Viewer.html').write_text(s)
notes='''# Side-printing rods — latest revision

Use Rods and guides print layout.stl for the four changed parts. The new locking rod REQUIRES the revised left and right bearing walls. Both rods print on their −Z sides: carriage rod Z=28.2, locking rod Z=40. Both lap ends, the complete sliding edge and locking-rod shoulder feet sit directly on the bed. There are no raised end undersides requiring support.

The locking rod's reinforced underside has been continued from Z=40 along its length; its previous ordinary lower edge was Z=42. This retains the cam floor thickness rather than shaving away the reinforcement. The module envelope, rod lengths, pin centres, cam surface, stop faces in X, row spacing and common Y height remain unchanged. The guide openings extend 2 mm farther in −Z with the same 0.35 mm guide-pad clearance; the minimum solid web between the two rod openings is 2.9 mm through a 4 mm wall.

The horizontal pin bores keep their circular pin clearance. Their roofs use 45-degree shoulders and short flat bridges: 2.07 mm for ordinary holes, 2.28 mm for the rear relief. No support is intended in these holes. Bridge quality still depends on printer calibration. Preserve the supplied orientation; do not rotate the rods onto their broad Y faces.

Current checks: Rod side-print checks.json, Rod side-print changes.json, Clearance checks.json, Carriage assembly access.json. Both rods and walls are watertight single solids and have no detached islands in the 0.2 mm screen. Cam geometry is unchanged, both end laps contact the bed, and end-to-end rods remain clear at the 80 mm module pitch. These are geometry checks, not load/fatigue qualification.

'''
(O/'Print notes.md').write_text(notes+(T/'Print notes.md').read_text())
(O/'README.md').write_text('''# Side-printable rods

Latest changes: flat printing faces for both rods; retained cam reinforcement extended along the locking rod; corresponding bearing-wall guides lowered 2 mm; short bridged roofs for horizontal pin holes. Use Rods and guides print layout.stl for all four replacement parts. The thicker locking rod cannot use the previous walls.

The previous worm-clearance, support-free worm-contact faces, strengthened actuator cheeks and continuous locking pockets remain unchanged. The saved printing snapshot is untouched. before-rod-side-print preserves the immediately preceding version.

In the viewer select Both rods or Rods and bearing walls. Bed orientations shows the actual exported orientations. Print notes.md describes checks and remaining limitations. Sources: rod-side-print/build.py, finish.py, package.py and publish.py, with geometry jobs run through work/run_queued_cool_job.py. Earlier per-part reports apply only to unchanged parts; Rod side-print checks.json supersedes prior rod/wall print orientations.
''')
keep=['Viewer.html','Model.json','Carriage control rod.stl','Lock control rod.stl','Left bearing wall.stl','Right bearing wall.stl','Carriage control rod print.stl','Lock control rod print.stl','Left bearing wall print.stl','Right bearing wall print.stl','Rods side-print layout.stl','Rods and guides print layout.stl','Rods and guides layout checks.json','Rod print orientations.png','Print layout.stl','Print layout checks.json','Print notes.md','README.md','Review views.png','Clearance checks.json','Carriage assembly access.json','Rod side-print changes.json','Rod side-print checks.json','Bed orientation checks.json']
for n in keep:shutil.copy2(O/n,T/n)
(T/'Current geometry hashes.json').write_text(json.dumps({n:hashlib.sha256((T/n).read_bytes()).hexdigest() for n in keep},indent=2));print('Published side-print rods',flush=True)
