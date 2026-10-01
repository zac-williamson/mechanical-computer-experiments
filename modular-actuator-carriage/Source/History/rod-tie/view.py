from pathlib import Path
import os,runpy,json
R=Path(__file__).resolve().parents[1];O=R/'rod-tie-candidate'
p=O/'Viewer.html';s=p.read_text().replace('Improved locking bolt guidance','Compact carriage — keyed rod connection').replace('Longer guided bolt shoulder · flared pocket entrances · three matching replacement parts.','Raised joining-pin mounts removed · two rod pins and locating keys secure the carriage halves.')
s=s.replace('<option value="rod-group">Both rods</option>','<option value="rod-tie-group">Carriage and keyed rod</option><option value="rod-group">Both rods</option>')
s=s.replace("const groups={","const groups={'rod-tie-group':['Carriage body','Carriage bearing end','Carriage control rod','Control rod attachment pin -11.0','Control rod attachment pin 11.0'],")
s=s.replace('<a href="Lock%20improvement%20print%20layout.stl">Lock replacement set STL</a>','<a href="Rod%20tie%20print%20layout.stl">Compact carriage replacement STL</a>').replace('<a href="Lock%20revision%20notes.md">Lock revision notes</a>','<a href="Rod%20tie%20notes.md">Keyed rod revision notes</a>')
p.write_text(s)
os.environ['PLANAR_OUTPUT']=str(O);runpy.run_path(str(R/'render_review.py'),run_name='__main__')
# A separate isolated preview for inspecting the connection, using actual meshes.
t=(R/'render_review.py').read_text().replace('idx=4;tri=[];cols=[]',"D['parts']=[p for p in D['parts'] if p['name'] in ['Carriage body','Carriage bearing end','Carriage control rod','Control rod attachment pin -11.0','Control rod attachment pin 11.0']]\nidx=4;tri=[];cols=[]").replace("'Review views.png'","'Rod connection views.png'")
exec(compile(t,'connection render','exec'),{'__file__':str(R/'render_review.py')})
