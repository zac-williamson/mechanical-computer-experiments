from pathlib import Path
import json,hashlib,ast,re,subprocess,zipfile
r=Path.cwd();o=r/'Wall register';h=hashlib.sha256((o/'geometry.npz').read_bytes()).hexdigest()
checks={'Carriage section checks':'carriage_sections_pass','Band installation checks':'band_installation_pass','Revision connection checks':'connection_pass','Manufacturing revision checks':'manufacturing_revision_pass','Assembly print orientations':'export_pass','Axle print-face audit':'all_axle_bed_faces_pass','Frame pin clearance checks':'pin_clearance_pass','Rod splice checks':'rod_splice_pass','Rod splice print checks':'splice_print_pass','Frame print checks':'frame_print_geometry_pass','Frame rebuild verification':'geometry_reproduction_pass','Development checks':'printed_motion_pass','Bearing attachment checks':'attachment_pass','Bearing print orientations':'orientation_pass','Bearing and retention checks':'support_layout_pass','Rotation ratio checks':'rotation_identity_pass','External gear phase checks':'external_profile_pass','Elastic anchor checks':'anchor_presence_pass'}
for n,k in checks.items():
 d=json.loads((o/(n+'.json')).read_text());assert d['geometry_sha256']==h and d[k],(n,k)
d=json.loads((o/'Axle crossing screening.json').read_text());assert d['geometry_sha256']==h and not d['potential_crossings']
d=json.loads((o/'Rotating envelope screening.json').read_text());assert d['geometry_sha256']==h and {a['native'] for a in d['potential_collisions']}=={'master U022','slave U022','Control clock U022','Control write U022'}
for p in (r/'Source').glob('*wall*.py'):ast.parse(p.read_text())
scripts=re.findall(r'<script[^>]*>(.*?)</script>',(r/'Wall register.html').read_text(),re.S)
Path('/tmp/wall-viewer-check.js').write_text(scripts[-1]);subprocess.run(['node','--check','/tmp/wall-viewer-check.js'],check=True)
# Run wall_status.py immediately before packaging.
assert json.loads((o/'Qualification status.json').read_text())['all_reports_current']
files=list(o.rglob('*'))+[r/'Wall register.html',r/'README.md',r/'AGENTS.md']+list((r/'Source').glob('*wall*.py'))+list((r/'Source').glob('wall_*.js'))+[r/'Source'/n for n in ['assembly.html','compact_pose.py','compact_elastic.py','ldraw_mesh.py']]
with zipfile.ZipFile(r/'Wall register development package.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in files:
  if p.is_file():z.write(p,p.relative_to(r))
 z.writestr('READ FIRST.txt','Development candidate, not a print release. Read Wall register/Assembly and print revision.md for the required rod/carrier installation sequence, 3L cheek pins and print folders. All mechanism fasteners are LEGO friction pins, without screws. Frames retain the two-piece pinned bit seam. Exported axle-bearing parts have verified bed-face rings. Static non-running overhangs may remain. CAD checks do not establish printed fit, strength or loaded operation. Start with one bit. Rebuilding requires baseline Compact layout and LDraw, not bundled.\n')
print('Current geometry, attachment, bed-face and clearance gates passed; development package rebuilt.',h)
