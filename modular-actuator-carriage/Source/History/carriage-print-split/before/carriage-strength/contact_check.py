from pathlib import Path
r=Path(__file__).resolve().parents[1]
s=(r/'validate.py').read_text()
s=s.replace("parts.append(p)","""if p['name'] in ['Carriage body','Carriage bearing end']:
  old=trimesh.load(R.parent/'carriage-strength/before'/(p['name']+'.stl'))
  p['s']-=solid(old)
 parts.append(p)""")
s=s.replace("if a['kind']!='printed' and b['kind']!='printed':continue", "if not ({a['name'],b['name']} & {'Carriage body','Carriage bearing end'}):continue")
s=s.replace("if 'Actuator lever' in names and 'Carriage body' in names:continue", "# Newly added material may not occupy the lever sweep.")
s=s.replace("if 'L099' in names and any(n in names for n in ['Carriage body','Carriage bearing end']):continue", "# Newly added material may not occupy the clutch ring.")
s=s.replace("Clearance checks.json","Added carriage material checks.json")
exec(compile(s,'added-contact-check','exec'))
