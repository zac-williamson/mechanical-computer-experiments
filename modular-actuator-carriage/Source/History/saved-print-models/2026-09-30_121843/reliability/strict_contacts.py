from pathlib import Path
import os
R=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R/'reliability-candidate')
s=(R/'validate.py').read_text()
s=s.replace("if 'Actuator lever' in names and 'Carriage body' in names:continue",'# Audit the retained lever contact rather than exempting the whole pair.')
s=s.replace("if any(n in names for n in ['U022','reaction-stop-axle','reaction-retainer','pivot-stop-axle','pivot-retainer']) and any(n in names for n in ['Upper actuator cheek','Lower actuator cheek','Actuator lever']):continue",'# Audit source bearing contacts.')
s=s.replace("if 'L099' in names and any(n in names for n in ['Carriage body','Carriage bearing end']):continue",'# Audit actual clutch-ring / printed-fork contacts.')
s=s.replace("Clearance checks.json","Strict contacts.json")
exec(compile(s,'strict contacts','exec'),{'__name__':'__main__'})
