from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R0/'simplified-carriage-r1')
source=(R0/'validate.py').read_text();prefix=source.split('printed_hits=[]')[0];exec(compile(prefix,'load mechanism','exec'))
baseparts=parts.copy();candidate_names=[]
for label,xa,yb in [('deeper',12,20),('wider',9.1,17.5),('middle',10.5,17.5)]:
 for sign in [-1,1]:
  a,b=(-16.25,-xa) if sign<0 else (xa,16.25)
  s=box([a,14.4,19.5],[b,yb,33])
  s-=cy(2.5,14.3,22.5,1,[sign*11,0,32])+cy(3.25,14.3,15.1,1,[sign*11,0,32])
  name=label+str(sign);candidate_names.append(name)
  parts.append(dict(name=name,s=s,motion='carriage',kind='printed',mates=[]))
tail=source[source.index('printed_hits=[]'):]
tail=tail.replace("if a['kind']!='printed' and b['kind']!='printed':continue", "if a['name'] not in candidate_names and b['name'] not in candidate_names:continue\n   if a['name'] in candidate_names and b['name'] in candidate_names:continue\n   other=b if a['name'] in candidate_names else a\n   if other['name'] in ['Carriage body','Carriage bearing end'] or other['name'].startswith('Control rod attachment pin'):continue")
tail=tail.replace("(R/'Clearance checks.json')","(R0/'neck-study/More space checks.json')")
exec(compile(tail,'space checks','exec'))
