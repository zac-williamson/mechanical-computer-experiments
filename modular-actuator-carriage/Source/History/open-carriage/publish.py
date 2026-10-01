from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];O=R/'open-carriage-candidate';T=R/'neck-candidate';B=T/'before-open-carriage';B.mkdir(exist_ok=True)
c=json.loads((O/'Clearance checks.json').read_text());assert all(not c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
assert json.loads((O/'Complete actuator assembly checks.json').read_text())['passed']
assert json.loads((O/'Detent function checks.json').read_text())['passed']
for p in T.iterdir():
 if p.is_file() and not (B/p.name).exists():shutil.copy2(p,B/p.name)
s=(O/'Viewer.html').read_text();tag='<script type="application/json" id="data">';start,rest=s.split(tag,1);raw,rest=rest.split('</script>',1);d=json.loads(raw);d['labels']['Carriage joining pin 0.5']='Lower front carriage pin';d['labels']['Carriage joining pin 32']='Upper front carriage pin';s=start+tag+json.dumps(d,separators=(',',':'))+'</script>'+rest
s=s.replace('<h1>Actuator, carriage and locking bolt</h1>','<h1>Open carriage — actuator assembly access</h1>').replace('Adapted from the planar register · two carriage pieces · four rows at 16 mm pitch.','Rear frame removed from the right half · open interior · joining pins at the front. Use “Assembly sequence” to inspect fitting around the built actuator.')
s=s.replace('<div class="tools"><label><input id="thin-strips"','<div class="tools" hidden><label><input id="thin-strips"').replace('id="thin-strips" type="checkbox" checked','id="thin-strips" type="checkbox"').replace("if(D.tiling)setmode('tiles');else draw();","setmode('assembly');")
s=s.replace('<div class="tools" id="motion">','<div class="tools"><button id="assembly-fit">Assembly sequence</button></div><div class="tools" id="fit-tools" hidden><label>Fit around the complete actuator <input id="fit-progress" type="range" min="0" max="100" step="1" value="0"></label></div><div class="tools" id="motion">',1)
s=s.replace('let upperIndex=D.frames.length-1;',"let upperIndex=D.frames.length-1;let fitPreview=false,fitProgress=0;")
fitcode="""
const fitNames=new Set(['Carriage body','Carriage bearing end','Carriage joining pin 0.5','Carriage joining pin 32','Carriage control rod','Control rod attachment pin -11.0','Control rod attachment pin 11.0','Upper actuator cheek','Lower actuator cheek','Actuator lever','U022','U015','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer','Actuator cheek joining pin 31.4','Actuator cheek joining pin 39.4','Actuator return band']);
function fitTransform(p){const m=identity.slice(),clamp=x=>Math.max(0,Math.min(1,x));if(['Carriage body','Carriage joining pin 0.5','Carriage joining pin 32'].includes(p.name))m[13]=-55*(1-clamp(fitProgress/30));if(p.name==='Carriage bearing end')m[12]=-55*(1-clamp((fitProgress-30)/30));if(p.name==='Carriage control rod')m[13]=-55*(1-clamp((fitProgress-75)/25));if(p.name.startsWith('Control rod attachment pin'))m[13]=-20*(1-clamp((fitProgress-60)/15));return m;}
function startAssemblyFit(){setmode('assembly');fitPreview=true;fitProgress=0;$('fit-progress').value=0;$('fit-tools').hidden=false;$('motion').hidden=true;$('explode-tools').hidden=true;$('frame').checked=true;$('carriage').checked=true;explosion=0;draw();}
$('assembly-fit').onclick=startAssemblyFit;$('fit-progress').oninput=e=>{fitProgress=+e.target.value;draw()};
"""
s=s.replace('function scene(){',fitcode+'\nfunction scene(){')
s=s.replace("const radius=mode==='print'?130:","const radius=fitPreview?105:mode==='print'?130:")
s=s.replace("for(const [dx,dz,pose] of copies)for(const p of scene()){","for(const [dx,dz,pose] of copies)for(const p of scene()){\n  if(fitPreview&&!fitNames.has(p.name))continue; if(fitPreview&&fitProgress<60&&p.name.startsWith('Control rod attachment pin'))continue;")
s=s.replace("const actual=mode!=='print'&&D.bands[pose][p.name]||p;","const actual=fitPreview?p:(mode!=='print'&&D.bands[pose][p.name]||p);")
s=s.replace("const matrix=mode==='tiles'?","const matrix=fitPreview?fitTransform(p):mode==='tiles'?")
s=s.replace(";$('legend').textContent=mode==='print'?",";if(fitPreview)$('readout').textContent=fitProgress<30?'1. Right half approaches from −Y; joining pins already fitted':fitProgress<60?'2. Left half slides in from −X onto the two joining pins':fitProgress<75?'3. Preload the rod attachment pins into the carriage from −Y':fitProgress<100?'4. Control rod slides from −Y onto the exposed pins':'Carriage fitted around the complete actuator. Insert the input axle next; fit bolt, guides and bearing walls afterwards.';$('legend').textContent=mode==='print'?")
s=s.replace("function setmode(m){","function setmode(m){fitPreview=false;$('fit-tools').hidden=true;")
s=s.replace('window.modelViewer={draw,','window.modelViewer={fitTransform,startAssemblyFit,draw,')
s=s.replace("'<option value=\"all\">Whole module</option>';for(const p of scene())", "'<option value=\"all\">Whole module</option><option value=\"worm-group\">Carriage and worm</option><option value=\"rod-group\">Both rods</option><option value=\"guide-group\">Rods and bearing walls</option>';for(const p of scene())")
s=s.replace("if($('part').value!=='all'&&p.name!==$('part').value)continue;", "const groups={'worm-group':['Carriage body','Carriage bearing end','U015'],'rod-group':['Carriage control rod','Lock control rod'],'guide-group':['Carriage control rod','Lock control rod','Left bearing wall','Right bearing wall']};if(groups[$('part').value]){if(!groups[$('part').value].includes(p.name))continue;}else if($('part').value!=='all'&&p.name!==$('part').value)continue;")
s=s.replace('<a href="2x2%20frame','<a href="Carriage%20print%20layout.stl">Replacement carriage STL</a> · <a href="Print%20layout.stl">Updated full layout</a> · <a href="Assembly%20instructions.md">Assembly order</a> · <a href="2x2%20frame',1)
s=s.replace('The carriage split places the clutch-contact face on the bed. Test the bearing-shoulder bridges on the supplied small sample before printing a full carriage.','Both worm-contact faces still print upward from solid material, with vertical bores. The assembly sequence keeps the actuator built; hold the lever near its neutral position and fit the input axle after the carriage halves.')
(O/'Viewer.html').write_text(s)
(O/'Assembly instructions.md').write_text('''# Fit the carriage around the preassembled actuator

1. Assemble the reaction gear, lever, upper and lower cheeks, their axles, retaining bushes and cheek joining pins. Keep the lever near its neutral position. Leave the worm input axle out for now.
2. Fit the two existing 2L carriage joining pins into the right/main carriage half from its exposed −X joint face. These holes are now at the front, Y = −1.2, Z = 12 and 22.5. Leave their left ends exposed.
3. Position the worm beside the reaction gear, then bring the right/main carriage half toward it from −Y. The rear cage no longer passes around the cheek mounting towers.
4. Bring the left/bearing-end carriage half from −X. It slides onto the two exposed joining pins. Its lever-contact arm and both locking pockets remain one piece.
5. Insert the two rod attachment pins into the carriage from −Y first. Then bring the control rod from −Y onto their exposed ends. Their centre collars do not pass through the rod holes.
6. Insert the worm input axle along X, then fit the remaining bolt, guides and bearing walls.

The viewer's Assembly sequence slider shows steps 3–5. It holds the complete actuator in place at the neutral assembly position. The worm is shown in its intended position; hold it there until the input axle is fitted. The frame and unrelated parts are hidden to make the fitting paths visible.

The checked paths include both cheeks, their joining pins, reaction gear, lever, axle stops, retaining bushes and return-band envelope. Each straight approach was sampled every 0.5 mm. This is an insertion-geometry check, not a physical assembly or force test.
''')
changes=json.loads((O/'Open carriage changes.json').read_text());old=sum(r['old_volume_mm3'] for r in changes);new=sum(r['new_volume_mm3'] for r in changes)
(O/'README.md').write_text(f'''# Open carriage

The right/main carriage rear cage has been removed. The left half has a large rectangular interior opening and no obsolete lower pin plate. The two carriage joining pins now sit in a front spine, so the right half can approach from −Y and the left half from −X around the complete actuator. Overall carriage plastic volume is reduced by {100*(old-new)/old:.1f}%. The added front spine extends 7.95 mm farther in −Y, within the previous 8 mm depth allowance; module pitch and external port coordinates are unchanged.

Use Assembly sequence in the viewer and read Assembly instructions.md. Carriage print layout.stl contains the two replacement halves; Print layout.stl contains the complete current module. Both worm-contact faces retain their upward-facing print orientation. Rods, walls, actuator parts and locking mechanism are unchanged.

Checks: Complete actuator assembly checks.json, Open carriage functional checks.json, Clearance checks.json and Detent function checks.json. Earlier carriage assembly checks describe the superseded split and should not be used. The current checks preserve the worm end bearings, clutch contact, lever profile, rod pin surrounds, reinforced bearing roots and unsplit locking pockets. Printing and loaded operation still require physical testing.

Sources: open-carriage/build.py, assembly.py, finish.py and publish.py. Run geometry work through work/run_queued_cool_job.py. before-open-carriage retains the prior published version; saved printing snapshots are untouched.
''')
(O/'Print notes.md').write_text('''# Open carriage revision

Use the new Carriage print layout.stl for both carriage halves. The right/main body prints on its outer +X face; the left/end half prints on its outer −X face. Both worm bores remain vertical, with upward-facing contact surfaces and solid material underneath. The new front joining bores are parallel to them. Neither worm-contact surface needs support. Other overhangs still require slicer inspection.

The front joining pins have moved to Y = −1.2 at Z = 12 and 22.5. Their count and type remain the same. Follow Assembly instructions.md; the old straight approach from +X is superseded by the demonstrated −Y / −X sequence.

Current meshes pass watertightness, single-solid and 0.2 mm layer-connectivity checks. Critical bearing, rod attachment, clutch, lever and lock surfaces are preserved. The previous rod and bearing-wall updates remain current, including side-printing rods and the widened locking-rod guide. No mechanical life or force claim is made from mesh checks.
''')
keep=['Viewer.html','Model.json','Carriage body.stl','Carriage bearing end.stl','Carriage body print.stl','Carriage bearing end print.stl','Carriage print layout.stl','Print layout.stl','Print layout checks.json','Bed orientation checks.json','Review views.png','Clearance checks.json','Complete actuator assembly checks.json','Open carriage functional checks.json','Open carriage changes.json','Detent function checks.json','Assembly instructions.md','README.md','Print notes.md']
for n in keep:shutil.copy2(O/n,T/n)
(T/'Current geometry hashes.json').write_text(json.dumps({n:hashlib.sha256((T/n).read_bytes()).hexdigest() for n in keep},indent=2));print('Published open carriage',flush=True)
