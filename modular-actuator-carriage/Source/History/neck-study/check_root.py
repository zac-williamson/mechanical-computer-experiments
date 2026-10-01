from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];os.environ['PLANAR_OUTPUT']=str(R0/'simplified-carriage-r1')
source=(R0/'validate.py').read_text();prefix=source.split('printed_hits=[]')[0];exec(compile(prefix,'load mechanism','exec'))
baseparts=parts.copy();candidate_names=[]
for label,xa,yb in [('stepped',9.1,22.4)]:
 for sign in [-1,1]:
  a,b=(-16.25,-xa) if sign<0 else (xa,16.25)
  s=box([a,13.5,24.8],[b,yb,27.7])
  s-=cy(2.5,14.3,22.5,1,[sign*11,0,32])+cy(3.25,14.3,15.1,1,[sign*11,0,32])
  name=label+str(sign);candidate_names.append(name)
  parts.append(dict(name=name,s=s,motion='carriage',kind='printed',mates=[]))
tail=source[source.index('printed_hits=[]'):]
tail=tail.replace("if a['kind']!='printed' and b['kind']!='printed':continue", "if a['name'] not in candidate_names and b['name'] not in candidate_names:continue\n   if a['name'] in candidate_names and b['name'] in candidate_names:continue\n   other=b if a['name'] in candidate_names else a\n   if other['name'] in ['Carriage body','Carriage bearing end'] or other['name'].startswith('Control rod attachment pin'):continue")
tail=tail.replace('if vol>.04:', 'if vol>.00001:')
tail=tail.replace("(R/'Clearance checks.json')","(R0/'neck-study/Root space checks.json')")
exec(compile(tail,'space checks','exec'))

O=R0/'neck-candidate';O.mkdir(exist_ok=True)
import shutil
for n in ['Carriage body','Carriage bearing end','Carriage control rod']:shutil.copy2(R/(n+'.stl'),O/(n+'.stl'))
rows=[]
for sign,n in [(-1,'Carriage bearing end'),(1,'Carriage body')]:
 old=next(p['s'] for p in baseparts if p['name']==n);p=next(p['s'] for p in parts if p['name']=='stepped'+str(sign));new=old+p
 mm=new.to_mesh64();t=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=True);t.export(O/(n+'.stl'))
 rows.append(dict(part=n,added_mm3=(new-old).volume(),solids=len(new.decompose())))
 for h in [14.4,17.5,20,22]:
  region=box([-16.25 if sign<0 else 9.1,h-.005,24.8],[-9.1 if sign<0 else 16.25,h+.005,27.7]);rows.append(dict(part=n,Y=h,old_section_mm2=(old^region).volume()/.01,new_section_mm2=(new^region).volume()/.01))
(O/'Section comparison.json').write_text(json.dumps(rows,indent=2))
assembly=(R0/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'neck-candidate'")
exec(compile(assembly,'candidate insertion checks','exec'),{'__name__':'__main__','__file__':str(R0/'carriage-print-split/check_assembly.py')})
