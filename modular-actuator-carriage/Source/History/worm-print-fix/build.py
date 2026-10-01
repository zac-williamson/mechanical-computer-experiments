from pathlib import Path
import json,os,shutil,runpy,ast,sys,base64,gzip
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];A=R0/'neck-candidate/before-worm-print-fix';O=R0/'worm-candidate';O.mkdir(exist_ok=True)
for f in A.iterdir():
 if f.is_file():shutil.copy2(f,O/f.name)
os.environ['PLANAR_OUTPUT']=str(O);source=(R0/'validate.py').read_text();exec(compile(source.split('printed_hits=[]')[0],'load mechanism','exec'))
sys.path.insert(0,str(R0.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R0/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
a=solid(trimesh.load(A/'Carriage body.stl'));b=solid(trimesh.load(A/'Carriage bearing end.stl'))
# Move the original lever-contact arm as a solid strip to the bearing end.
region=box([-1.81,29.5,11.6],[26.7,40.5,20.0])
bridge=(b^box([-1.82,29.5,11.6],[-1.801,40.5,20.0])).translate([1.82,0,0]).scale([12,1,1]).translate([-1.82,0,0])
end=b+(a^region)+bridge
body=a-box([-1.82,29.3,11.4],[26.7,40.7,20.2])
# Open the space around the full rotating worm envelope. Keep its end bearings.
clear=box([-9.1,4.2,9.3],[9.1,18.0,23.5])
body-=clear;end-=clear
report={'worm_native_radius_mm':5.009961,'open_space_Y':[4.2,18.0],'open_space_Z':[9.3,23.5],'bearing_thrust_faces_X':[-9.1,9.1],'lever_profile_owner':'Carriage bearing end','joint_clearance_mm':.2}
for n,s,bed in [('Carriage body',body,'right'),('Carriage bearing end',end,'left')]:
 print('components',n,[(c.volume(),c.bounding_box()) for c in s.decompose()],flush=True)
 env['add'](n,s,motion='carriage',bed=bed);env['parts'][-1]['mesh'].export(O/(n+'.stl'))
report['added_mm3']=(end+body-a-b).volume();report['removed_mm3']=(a+b-end-body).volume()
(O/'Worm print fix.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=trimesh.load(O/(p['name']+'.stl')).triangles.reshape(-1,3) if p['kind']=='printed' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
 if p['name']=='Carriage bearing end':p['bed']='left'
 if p['name']=='Carriage body':p['bed']='right'
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
shutil.copy2(R0/'simplified-carriage-r1/Development checks.json',O/'Development checks.json')
for script in ['validate.py','package_preview.py']:runpy.run_path(str(R0/script),run_name='__main__')
s=(R0/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'worm-candidate'");exec(compile(s,'assembly','exec'),{'__name__':'__main__','__file__':str(R0/'carriage-print-split/check_assembly.py')})
print('BUILD FINISHED',flush=True)
