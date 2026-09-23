from pathlib import Path
import json,hashlib,zipfile
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
ps=json.loads((O/'printed-parts.json').read_text());hs=json.loads((O/'hardware.json').read_text());op=json.loads((O/'Operation checks.json').read_text());hw=json.loads((O/'Operation hardware checks.json').read_text());rot=json.loads((O/'Rotating clearance checks.json').read_text());mesh=json.loads((O/'Criteria audit evidence.json').read_text())
assert not op['printed_interferences']
assert not [r for r in hw['printed_native_checks'] if r['max_sampled_surface_interior_mm']>.03 and 'pin' not in r['pair'][1].lower()]
assert not rot['penetrations'],rot['penetrations']
assert all(r['print']['watertight'] and r['print']['components_including_open']==1 for r in mesh['print_meshes'])
files=[O/'Prototype print parts'/(p['id']+'.stl') for p in ps]
files += [O/n for n in ['Prototype print parts/Orientations.json','README.md','Cam mechanism audit.md','Loaded guide screen.json','Bolt guide play.json','Transition timing audit.md','Transition timing audit.json','Clutch phase contact scan.json','SAP register timing.md','Criteria audit.md','Operation checks.json','Operation hardware checks.json','Contact classification.json','Rotating clearance checks.json','Actuator band checks.json','Qualification.json','Carriage printing checks.json','Criteria audit evidence.json','Flex screening.json','Memory gear mesh.json','WRITE gear mesh.json','Actuator addition checks.json','Direct cam parameters.json','Viewer.html','Additional LEGO parts.csv','printed-parts.json','hardware.json']]
manifest=dict(printed_parts=len(ps),modelled_hardware_parts=len(hs),required_torque_Nm=.1,qualified=False,functional_transition_status='Bare latch fails arbitrary simultaneous close/data reversal; external SAP capture sequencing unqualified',source_commit='5d987d9d05220db57576819dd6835af9f23af557',files=[dict(path=str(p.relative_to(O)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files]);(O/'Current package manifest.json').write_text(json.dumps(manifest,indent=2));files.append(O/'Current package manifest.json')
with zipfile.ZipFile(O/'Direct cam register prototype.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in files:z.write(p,p.relative_to(O))
print('Packaged',len(ps),'print parts;',len(hs),'hardware entries. Sampled geometry checks retained; functional transition qualification fails for unsequenced close/data reversal.')
