from pathlib import Path
import json,os,shutil,runpy,ast,sys,base64,gzip
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];A=R0/'neck-candidate';O=R0/'detent-candidate';O.mkdir(exist_ok=True)
for f in A.iterdir():
 if f.is_file():shutil.copy2(f,O/f.name)
os.environ['PLANAR_OUTPUT']=str(O);source=(R0/'validate.py').read_text();exec(compile(source.split('printed_hits=[]')[0],'load mechanism','exec'))
sys.path.insert(0,str(R0.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R0/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
a=solid(trimesh.load(A/'Carriage body.stl'));b=solid(trimesh.load(A/'Carriage bearing end.stl'))
# The complete keeper belongs to the end. Its extension inserts along X.
region=box([-1.81,20.5,30.1],[4.1,31.5,39.41])
# Extrude the existing split section, preserving the pocket void through it.
bridge=(b^box([-1.82,20.5,30.1],[-1.801,31.5,39.41])).translate([1.82,0,0]).scale([12,1,1]).translate([-1.82,0,0])
end=b+(a^region)+bridge
body=(a-box([-1.82,20.3,29.9],[4.3,31.7,39.6]))+box([-1.6,22.2,26.7],[4.4,24.2,29.9])
report={'joint_clearance_mm':.2,'keeper_owner':'Carriage bearing end','pockets':[]}
for x in [-8.8,-1.3]:
 # Each complete stop face and floor must be in a single part.
 pocket=box([x-2.14,23.51,35.81],[x+2.14,28.49,39.5])
 assert (end^pocket).volume()<.001
 for side in [-1,1]:
  probe=box([x+side*2.15-(.8 if side<0 else 0),24,36],[x+side*2.15+(0 if side<0 else .8),28,39])
  assert (end^probe).volume()>9.5
 report['pockets'].append({'center_X':x,'both_stops_in_one_part':True})
for n,s,bed in [('Carriage body',body,'left'),('Carriage bearing end',end,'right')]:
 env['add'](n,s,motion='carriage',bed=bed);env['parts'][-1]['mesh'].export(O/(n+'.stl'))
report['added_mm3']=(end+body-a-b).volume();report['removed_mm3']=(a+b-end-body).volume()
(O/'Detent joint checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=trimesh.load(O/(p['name']+'.stl')).triangles.reshape(-1,3) if p['kind']=='printed' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
 if p['name']=='Carriage bearing end':p['bed']='left'
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
shutil.copy2(R0/'simplified-carriage-r1/Development checks.json',O/'Development checks.json')
for script in ['validate.py','package_preview.py']:runpy.run_path(str(R0/script),run_name='__main__')
s=(R0/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'detent-candidate'");exec(compile(s,'assembly','exec'),{'__name__':'__main__','__file__':str(R0/'carriage-print-split/check_assembly.py')})
print('BUILD FINISHED',flush=True)
