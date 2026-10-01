from pathlib import Path
import json,os,shutil,runpy,ast,sys,base64,gzip
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];A=R0/'neck-candidate/before-open-carriage';O=R0/'open-carriage-candidate';O.mkdir(exist_ok=True)
for f in A.iterdir():
 if f.is_file():shutil.copy2(f,O/f.name)
os.environ['PLANAR_OUTPUT']=str(O);source=(R0/'validate.py').read_text();exec(compile(source.split('printed_hits=[]')[0],'load mechanism','exec'))
sys.path.insert(0,str(R0.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R0/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
a=solid(trimesh.load(A/'Carriage body.stl'));b=solid(trimesh.load(A/'Carriage bearing end.stl'))
# Retain the bearing/rod/fork structure, not the rear cage of the right half.
body=a^(box([-2,-20,-10],[27,17.5,50])+box([-2,-20,24.8],[27,22.4,50]))
body-=box([-1.7,4.2,23.5],[5.2,22.5,30.1])
# Broad rectangular opening under the reinforced root on the left half.
end=b-box([-17,17.5,5],[27,30,24.8])-box([-17,17.5,-10],[27,44,11.4])
# Join the halves on the accessible front, clear of the assembled actuator.
body+=box([-1.6,-3.2,12],[16.25,1.8,22.5])
end+=box([-16.25,-3.2,12],[-1.8,1.8,22.5])
for z in [12,22.5]:
 for which,xa,xb in [('body',-1.6,16.25),('end',-16.25,-1.8)]:
  boss=cy(5.2,xa,xb,0,[0,-1.2,z])
  if which=='body':body+=boss
  else:end+=boss
 bore=cy(2.5,-16.35,16.35,0,[0,-1.2,z])+cy(3.3,-2.1,-1.3,0,[0,-1.2,z])
 body-=bore;end-=bore
report=[]
for name,new,old,bed in [('Carriage body',body,a,'right'),('Carriage bearing end',end,b,'left')]:
 print('COMPONENTS',name,[(c.volume(),c.bounding_box()) for c in new.decompose()],flush=True)
 env['add'](name,new,motion='carriage',bed=bed);env['parts'][-1]['mesh'].export(O/(name+'.stl'))
 report.append(dict(part=name,removed_mm3=(old-new).volume(),added_mm3=(new-old).volume(),old_volume_mm3=old.volume(),new_volume_mm3=new.volume()))
(O/'Open carriage changes.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=trimesh.load(O/(p['name']+'.stl')).triangles.reshape(-1,3) if p['kind']=='printed' else v[p['offset']//3:p['offset']//3+p['vertices']]
 if p['name']=='Carriage joining pin 0.5':a=a+np.array([0,-25.9,11.5])
 if p['name']=='Carriage joining pin 32':a=a+np.array([0,-36.6,-9.5])
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
 if p['name']=='Carriage bearing end':p['bed']='left'
 if p['name']=='Carriage body':p['bed']='right'
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
shutil.copy2(R0/'simplified-carriage-r1/Development checks.json',O/'Development checks.json')
for script in ['validate.py','package_preview.py']:runpy.run_path(str(R0/script),run_name='__main__')
print('BUILD FINISHED',flush=True)
