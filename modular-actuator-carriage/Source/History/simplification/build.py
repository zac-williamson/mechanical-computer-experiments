from pathlib import Path
import json,os,shutil,runpy,base64,gzip,ast,sys
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'adapted';O=R/'simplified-carriage-r1';O.mkdir(exist_ok=True)
for f in A.iterdir():
 if f.is_file() and f.suffix not in ['.stl','.png'] or f.is_file() and f.suffix=='.stl':shutil.copy2(f,O/f.name)
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
revised={};report=[]
for name,lo,hi in [('Carriage body',-1.6,16.25),('Carriage bearing end',-16.25,-1.8)]:
 old=solid(trimesh.load(A/(name+'.stl')));s=old
 # Open narrow clearance remnants cleanly, with small trimming margins.
 for z in [11.6,20.2]:s-=box([-9,13.25,z-.05],[9,18,z+.25])
 # Replace the stepped pivot fringe with one rectangular clearance opening.
 s-=box([-4,20.85,26.7],[4,22.85,30.15])
 # Remove the tiny reaction-clearance shelf without entering the rod-pin grip.
 s-=box([13.49,22.399,28.19],[16.26,22.54,28.91])
 # Back the lever-contact structure with a continuous rear web; keep working face.
 s+=box([lo,37.7,11.6],[hi,40.4,20.0])
 if name=='Carriage bearing end':s+=box([-16.25,35.2,11.49],[-8.1,40.4,20.41])
 # Replace the layered central fork mount with one continuous saddle.
 def xcyl(radius):return m.Manifold.cylinder(16.2,radius,circular_segments=96).rotate([0,90,0]).translate([-8.1,10.2,0])
 region=box([-8,1.55,5.5],[7.8,13,11.2])
 protected=xcyl(7.4)
 saddle=box([-8,1.55,7.2],[7.8,13,11.2])-xcyl(7.6)
 worm=m.Manifold.cylinder(16.2,5.3,circular_segments=96).rotate([0,90,0]).translate([-8.1,10.2,16])
 saddle-=worm
 half=box([lo,-100,-100],[hi,100,100])
 before=s;root=box([-1.601,1.55,7.1],[1.601,13.2,11.2]);s=(s-(region-xcyl(7.65)-root))+(saddle^half)
 print('Fork components',name,[(c.volume(),c.bounding_box()) for c in s.decompose()],flush=True)
 assert ((before-s)^protected).volume()<.001,'Clutch contact changed'
 env['add'](name,s,'carriage','left' if lo>-2 else 'right');t=env['parts'][-1]['mesh'];t.export(O/(name+'.stl'));revised[name]=t
 new=solid(t);report.append(dict(part=name,removed_mm3=(old-new).volume(),added_mm3=(new-old).volume(),watertight=t.is_watertight))
D=json.loads((A/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arr=[];offset=0
for p in D['parts']:
 a=revised[p['name']].triangles.reshape(-1,3) if p['name'] in revised else v[p['offset']//3:p['offset']//3+p['vertices']];p['offset']=offset;p['vertices']=len(a);offset+=a.size;arr.append(a)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arr).astype('<f4').tobytes())).decode();(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
(O/'Simplification changes.json').write_text(json.dumps(report,indent=2));os.environ['PLANAR_OUTPUT']=str(O)
for script in ['carriage-print-split/audit_carriage.py','validate.py','package_preview.py']:runpy.run_path(str(R/script),run_name='__main__')
print('REVISION CHECKS COMPLETE',flush=True)
