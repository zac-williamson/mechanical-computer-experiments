"""Publish a fail-closed development status and per-instance model inventory."""
from pathlib import Path
import csv,json,hashlib
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
digest=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest()
reports={}
for name in ['Development checks','Rotation ratio checks','External gear phase checks','Rotating envelope screening','Axle crossing screening','Bearing and retention checks','Bearing attachment checks','Bearing print orientations','Axle print-face audit','Frame print checks','Frame pin clearance checks','Rod splice checks','Rod splice print checks','Frame rebuild verification','Elastic anchor checks','Force and mass review','Revision connection checks','Manufacturing revision checks','Assembly print orientations','Carriage section checks','Band installation checks']:
 path=O/(name+'.json')
 data=json.loads(path.read_text()) if path.exists() else {}
 reports[name]=dict(current=data.get('geometry_sha256')==digest,data=data)
ps=json.loads((O/'parts.json').read_text())
with (O/'Model inventory.csv').open('w',newline='') as f:
 w=csv.writer(f,lineterminator="\n");w.writerow(['instance','module','kind','native_part_or_inherited_source','one_bit_quantity','eight_bit_quantity','development_stl'])
 for p in ps:
  w.writerow([p['id'],p['module'],p['kind'],p.get('lego_part') or p.get('hardware') or p.get('source') or '',1,1 if p['module']=='control' else 8,'Development parts/'+p['id'].replace('/','-')+'.stl' if p['kind']=='printed' else ''])
blocks=[
 'The new shared linkage has not been dynamically or contact-resolved simulated under load. Imported original traces prescribe positions.',
 'Full native/native collision, worm contact, phase-resolved actuator gear/lever contact, retention and assembly-access qualification remains open.',
 'Elastic loops follow their inherited moving-anchor paths in the viewer; preload, retention and physical elastic contact are not qualified.',
 'Printed strength, rod buckling, tolerance, friction, wear, pin fit and eight-row actuation force have not been measured.',
 'The expanded printed screen uses two adjacent rows carrying the same inherited trace; independent bit states and loaded rod deflection remain unverified.',
 'Development parts STLs are in assembly coordinates. Use the separate oriented exports and documented rod/carrier assembly sequence; final slicer review and physical pin fit remain to be checked.',
 'Equal engaged gear ratios do not guarantee constant instantaneous RPM during reversal or disconnection.',
 'The revised keeper adds 6.8 mm to the left-hand bit envelope; frame height and 112 mm row pitch are unchanged.'
]
lines=['# Validation status — modular wall register','','**Development candidate; not a print release.**','',f'Geometry SHA-256: `{digest}`.','','| Screen | Current evidence |','|---|---|']
for name,item in reports.items():
 r=item['data'];status='MISSING OR STALE'
 if item['current']:
  if name=='Development checks':status=f"{'PASS' if r['printed_motion_pass'] else 'FAIL'}: {r['frames']} sampled unique poses; {len(r['intersections'])} printed intersections; two adjacent rows + controller"
  elif name=='Rotation ratio checks':status=f"{'PASS' if r['unit_magnitude_pass'] and r['rotation_identity_pass'] else 'FAIL'}: {len(r['meshes'])} discovered external meshes; unit magnitude"
  elif name=='External gear phase checks':status=f"{'PASS' if r['external_profile_pass'] else 'FAIL'}: sampled external tooth profiles (not worm/clutch qualification)"
  elif name=='Rotating envelope screening':status=f"{len(r['potential_collisions'])} potential contacts retained for review; {r['unique_poses']} poses across the 112 inherited operating cases"
  elif name=='Elastic anchor checks':status=f"{'PASS' if r['anchor_presence_pass'] else 'FAIL'}: {len(r['anchors'])} anchor centres present across all sampled operating cases"
  elif name=='Bearing and retention checks':status=f"{'PASS' if r['support_layout_pass'] else 'FAIL'}: {len(r['bearings'])} complete bearing lands; {len(r['retention'])} retained transmission assemblies"
  elif name=='Bearing attachment checks':status=f"{'PASS' if r['attachment_pass'] else 'FAIL'}: {len(r['removable_transmission_walls'])} removable transmission walls plus front/rear cheeks; two engaged pin axes each; two-piece frame joint verified; {len(r.get('removable_frame_fixtures',[]))} paired-pin fixture mounts"
  elif name=='Frame pin clearance checks':status=f"{'PASS' if r['pin_clearance_pass'] else 'FAIL'}: {r['pins']} friction-pin envelopes checked against other hardware and parts through operating poses"
  elif name=='Rod splice checks':status=f"{'PASS' if r['rod_splice_pass'] else 'FAIL'}: two pinned rod splices, two engaged pin sockets each"
  elif name=='Rod splice print checks':status=f"{'PASS' if r['splice_print_pass'] else 'FAIL'}: two splice bridges with pin holes normal to the bed and no unsupported faces beyond 45 degrees"
  elif name=='Frame rebuild verification':status=f"{'PASS' if r['geometry_reproduction_pass'] else 'FAIL'}: source rebuild reproduces the same solids and poses; triangle ordering may differ"
  elif name=='Frame print checks':status=f"{'PASS' if r['frame_print_geometry_pass'] else 'FAIL'}: {len(r['parts'])} flat-bed frame parts; no unsupported faces beyond 45 degrees; no bridge exemption"
  elif name=='Bearing print orientations':status=f"{'PASS' if r['orientation_pass'] else 'FAIL'}: {len(r['parts'])} wall STLs with bearing axes normal to the bed"
  elif name=='Axle print-face audit':status=f"{'PASS' if r['all_axle_bed_faces_pass'] else 'INCOMPLETE'}: {len(r['parts'])} parts screened; {len(r['unresolved_parts'])} parts need further separation or bed-face work"
  elif name=='Carriage section checks':status='PASS: eight carriage halves; reinforced corners and backing sections' if r['carriage_sections_pass'] else 'FAIL'
  elif name=='Band installation checks':status='PASS: closed-band insertion at both lock anchors' if r['band_installation_pass'] else 'FAIL'
  elif name in ['Revision connection checks','Manufacturing revision checks','Assembly print orientations']:
   key={'Revision connection checks':'connection_pass','Manufacturing revision checks':'manufacturing_revision_pass','Assembly print orientations':'export_pass'}[name];status='PASS' if r[key] else 'FAIL'
  elif name=='Axle crossing screening':status=f"{len(r['potential_crossings'])} potential axle crossings"
  else:status='Geometric printed-volume comparison and ideal force balance; no measured force or strength claim'
 lines.append(f'| [{name}]({name.replace(" ","%20")}.json) | {status} |')
lines+=['','## Scope','','The printed screen samples 17 positions from each of 112 inherited transition/initial-state traces, deduplicates identical printed poses, and adds a neighbouring row. It checks volumetric intersections above 0.01 mm³ without contact exclusions, plus watertightness and connected-solid count. It is a sampled screen, not a continuous swept-volume proof. The linkage equations separately cover 1,001 points across each nominal stroke.','', 'Full-revolution native envelopes are conservative. Their remaining actuator U022 / Short lever flags are reported rather than silently excluded; identifying an inherited working interface does not establish that it is correct in hardware.','', '## Open requirements','']+['- '+b for b in blocks]
(O/'Validation status.md').write_text('\n'.join(lines)+'\n')
status=dict(geometry_sha256=digest,parts_sha256=hashlib.sha256((O/'parts.json').read_bytes()).hexdigest(),all_reports_current=all(r['current'] for r in reports.values()),mechanically_qualified=False,print_release=False,open_requirements=blocks,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in list((R/'Source').glob('*wall*.py'))+list((R/'Source').glob('wall_*.js'))})
(O/'Qualification status.json').write_text(json.dumps(status,indent=2))
print(json.dumps({k:v for k,v in status.items() if k!='source_sha256'},indent=2))
