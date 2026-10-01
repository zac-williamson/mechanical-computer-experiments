from pathlib import Path
import json,os,shutil,runpy,ast,sys,base64,gzip
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];A=R0/'neck-candidate/before-rod-side-print';O=R0/'rod-candidate';O.mkdir(exist_ok=True)
for f in A.iterdir():
 if f.is_file():shutil.copy2(f,O/f.name)
os.environ['PLANAR_OUTPUT']=str(O);source=(R0/'validate.py').read_text();exec(compile(source.split('printed_hits=[]')[0],'load mechanism','exec'))
sys.path.insert(0,str(R0.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R0/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
def roof(r,x,z,ya,yb):
 # 45-degree shoulders with a short flat bridge, within the original bore height.
 d=r/np.sqrt(2);t=r*(np.sqrt(2)-1)
 return m.CrossSection([[[-d,z+d],[d,z+d],[t,z+r],[-t,z+r]]]).extrude(yb-ya).rotate([90,0,0]).translate([x,yb,0])
report=[]
for name,z in [('Carriage control rod',32),('Lock control rod',48)]:
 old=solid(trimesh.load(A/(name+'.stl')));s=old
 if name=='Lock control rod':
  # Continue the reinforced bottom to a single bed plane, including lap ends.
  extension=box([-47.8,6.2,40],[47.8,14.2,42.2])
  extension-=box([-48,6.1,39.9],[-32,10.6,42.3])+box([32,9.8,39.9],[48,14.3,42.3])
  for xa,xb in [(-16.65,-13.65),(13.65,16.65)]:extension+=box([xa,2.2,40],[xb,6.3,42.2])
  s+=extension
 for x in [-44,-36,36,44]+([-11,11] if name=='Carriage control rod' else []):s-=roof(2.5,x,z,6.1,14.3)
 for x in [-44,-36]:s-=roof(2.75,x,z,12.6,14.3)
 env['add'](name,s,motion='carriage' if z==32 else 'lock',bed='bottom');env['parts'][-1]['mesh'].export(O/(name+'.stl'))
 report.append(dict(part=name,added_mm3=(s-old).volume(),removed_mm3=(old-s).volume(),bed_Z=28.2 if z==32 else 40))
# The existing four-pad guides retain their side-pad positions. Extend only downward.
u=4.65;lo=8.65;hi=6.65;w=1.2;d=.3
poly=[[-u,-lo],[-w,-lo],[-w,-lo+d],[w,-lo+d],[w,-lo],[u,-lo],[u,-w],[u-d,-w],[u-d,w],[u,w],[u,hi],[w,hi],[w,hi-d],[-w,hi-d],[-w,hi],[-u,hi],[-u,w],[-u+d,w],[-u+d,-w],[-u,-w]]
for name,xa,xb,bed in [('Left bearing wall',-24.4,-20.4,'right'),('Right bearing wall',20.4,24.4,'left')]:
 old=solid(trimesh.load(A/(name+'.stl')));cut=m.CrossSection([poly]).extrude(xb-xa+.2).rotate([90,0,90]).translate([xa-.1,10.2,48]);s=old-cut
 env['add'](name,s,bed=bed);env['parts'][-1]['mesh'].export(O/(name+'.stl'))
 report.append(dict(part=name,removed_mm3=(old-s).volume(),minimum_web_between_rod_guides_mm=2.9))
(O/'Rod side-print changes.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=trimesh.load(O/(p['name']+'.stl')).triangles.reshape(-1,3) if p['kind']=='printed' else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
 if p['name'] in ['Carriage control rod','Lock control rod']:p['bed']='bottom'
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
shutil.copy2(R0/'simplified-carriage-r1/Development checks.json',O/'Development checks.json')
for script in ['validate.py','package_preview.py']:runpy.run_path(str(R0/script),run_name='__main__')
s=(R0/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'rod-candidate'");exec(compile(s,'assembly','exec'),{'__name__':'__main__','__file__':str(R0/'carriage-print-split/check_assembly.py')})
print('BUILD FINISHED',flush=True)
