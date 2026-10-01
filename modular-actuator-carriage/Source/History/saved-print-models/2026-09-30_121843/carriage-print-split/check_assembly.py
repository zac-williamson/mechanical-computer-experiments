from pathlib import Path
import os,runpy,json
R=Path(__file__).resolve().parents[1];O=R/'split-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R/'assembly-access/check.py').read_text().replace('b.translate([shift,0,0])','b.translate([-shift,0,0])').replace('Right half approach along -X','Left end approach along +X').replace("(Path(__file__).parent/('candidate.json' if '--candidate' in sys.argv else 'current.json'))","(O/'Split assembly insertion checks.json')")
s=s.split("if '--verify' in sys.argv:")[0]
exec(compile(s,'split-assembly-check','exec'))
pins=sum((cy(2.45,-9.7,6.3,0,[0,y,z])+cy(3.25,-2.1,-1.3,0,[0,y,z]) for y,z in [(24.7,.5),(35.4,32)]),m.Manifold())
for shift in np.linspace(16,0,33):
 for label,hit in [('Joining pins into left end',(pins.translate([shift,0,0])^b)),('Main body onto end and pins',(a.translate([shift,0,0])^(b+pins)))]:
  checks.append(dict(step=label,offset=float(shift),collision_mm3=hit.volume(),bounds=hit.bounding_box() if hit.volume()>.02 else None))
report=dict(checks=checks,passed=all(c['collision_mm3']<.02 for c in checks),scope='Carriage halves, joining pins, control rod and its pins. Not a full-module assembly proof.')
(O/'Carriage assembly access.json').write_text(json.dumps(report,indent=2));print('Split assembly failures',json.dumps([c for c in checks if c['collision_mm3']>=.02],indent=2),flush=True)
