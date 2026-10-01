from pathlib import Path
import json,gzip,base64,shutil,os,runpy
import numpy as np,trimesh
R=Path(__file__).resolve().parents[1];T=Path(__file__).resolve().parent;O=R/'split-candidate';O.mkdir(exist_ok=True)
D=json.loads((R/'carriage-print-split/before/Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
arrays=[];offset=0
for p in D['parts']:
 a=v[p['offset']//3:p['offset']//3+p['vertices']].copy()
 if p['kind']=='printed':
  src=T/('right.stl' if p['name']=='Carriage body' else 'left.stl') if p['name'] in ['Carriage body','Carriage bearing end'] else R/'carriage-print-split/before'/(p['name']+'.stl')
  shutil.copyfile(src,O/(p['name']+'.stl'))
  a=trimesh.load(src).triangles.reshape(-1,3)
  if p['name']=='Carriage body':p['bed']='left'
  if p['name']=='Carriage bearing end':p['bed']='right'
 if p['name'].startswith('Carriage joining pin'):a[:,0]-=9.9
 if p['name'].startswith('Control rod attachment pin'):
  p['mates']=['Carriage control rod','Carriage bearing end' if '-11' in p['name'] else 'Carriage body']
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arrays.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D))
os.environ['PLANAR_OUTPUT']=str(O)
runpy.run_path(str(R/'validate.py'),run_name='__main__')
s=(R/'validate.py').read_text().replace('parts.append(p)',"""if p['name'] in ['Carriage body','Carriage bearing end']:
  for oldname in ['Carriage body','Carriage bearing end']:
   p['s']-=solid(trimesh.load(R.parent/'carriage-print-split/before'/(oldname+'.stl')))
 parts.append(p)""")
s=s.replace("if a['kind']!='printed' and b['kind']!='printed':continue", "if not ({a['name'],b['name']} & {'Carriage body','Carriage bearing end'}):continue")
s=s.replace("if 'Actuator lever' in names and 'Carriage body' in names:continue",'# Check all new material against the lever.')
s=s.replace("if 'L099' in names and any(n in names for n in ['Carriage body','Carriage bearing end']):continue",'# Check all new material against the ring.')
s=s.replace('Clearance checks.json','Added carriage material checks.json');exec(compile(s,'split-contact-check','exec'),{'__name__':'__main__','__file__':str(R/'validate.py')})
runpy.run_path(str(R/'design-review/print_audit.py'),run_name='__main__')

runpy.run_path(str(T/"check_assembly.py"),run_name="__main__")
