from pathlib import Path
import json,os,shutil,runpy,ast,sys,base64,gzip
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];A=R0/'simplified-carriage-r1';C=R0/'neck-candidate';O=R0/'strength-candidate';O.mkdir(exist_ok=True)
for f in A.iterdir():
 if f.is_file():shutil.copy2(f,O/f.name)
for n in ['Carriage body','Carriage bearing end']:shutil.copy2(C/(n+'.stl'),O/(n+'.stl'))
os.environ['PLANAR_OUTPUT']=str(O);source=(R0/'validate.py').read_text();exec(compile(source.split('printed_hits=[]')[0],'load mechanism','exec'))
sys.path.insert(0,str(R0.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R0/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
report=[]
for n,z in [('Upper actuator cheek',20.4),('Lower actuator cheek',7.6)]:
 old=solid(trimesh.load(A/(n+'.stl')));web=box([31,21,z],[36.2,31.4,z+4]);web-=cy(2.5,z-.1,z+4.1,2,[35.25,31.4,0]);new=old+web
 if n=='Upper actuator cheek':
  for yy in [31.4,39.4]:new-=cy(2.5,28.3,29.05,2,[35.25,yy,0])
 env['add'](n,new,bed='bottom');t=env['parts'][-1]['mesh'];t.export(O/(n+'.stl'));report.append(dict(part=n,added_mm3=(solid(t)-old).volume(),removed_mm3=(old-solid(t)).volume()))
# Reinforce the guide's long front side walls outside the moving bolt slot.
n='Locking bolt guide';old=solid(trimesh.load(A/(n+'.stl')))
new=old+box([-13.45,20,40.5],[-11.45,25,47.2])+box([1.35,20,40.5],[3.35,25,47.2])
# Support the rear edge of the thin floor, behind the moving bolt envelope.
new+=box([-11.45,32,39.8],[1.35,32.6,42])
env['add'](n,new,bed='rear');t=env['parts'][-1]['mesh'];t.export(O/(n+'.stl'));report.append(dict(part=n,added_mm3=(solid(t)-old).volume(),removed_mm3=(old-solid(t)).volume()))
# Rebuild mesh data from current exported solids, retaining hardware and poses.
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=trimesh.load(O/(p['name']+'.stl')).triangles.reshape(-1,3) if p['kind']=='printed' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
(O/'Cheek reinforcement.json').write_text(json.dumps(report,indent=2))
for script in ['validate.py','package_preview.py']:runpy.run_path(str(R0/script),run_name='__main__')
print('BUILD FINISHED',flush=True)
