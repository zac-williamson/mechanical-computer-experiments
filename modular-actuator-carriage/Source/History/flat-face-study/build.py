from pathlib import Path
import os,json,shutil,ast,sys,runpy
import numpy as np,trimesh,manifold3d as m
R0=Path(__file__).resolve().parents[1];O=R0/'flat-face-candidate';O.mkdir(exist_ok=True)
A=R0/'neck-candidate';base=R0/'simplified-carriage-r1'
for n in ['Carriage control rod.stl']:shutil.copy2(base/n,O/n)
os.environ['PLANAR_OUTPUT']=str(base)
source=(R0/'validate.py').read_text();exec(compile(source.split('printed_hits=[]')[0],'load mechanism','exec'));report=[]
sys.path.insert(0,str(R0.parents[1]/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Source'))
tree=ast.parse((R0/'adapt.py').read_text());defs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['solid','mesh','add']];env=dict(np=np,trimesh=trimesh,m=m,OUT=O,parts=[]);exec(compile(ast.Module(body=defs,type_ignores=[]),'export','exec'),env)
for n,xa,xb,bed in [('Carriage body',-1.6,16.25,'left'),('Carriage bearing end',-16.25,-1.8,'right')]:
 old=solid(trimesh.load(A/(n+'.stl')))
 region=box([-16.26,1.54,4.0],[16.26,18.21,11.3])
 fork=old^box([-1.601,1.54,4],[1.601,13.21,11.3])
 slab=box([xa,1.55,9.3],[xb,12.7,11.3])
 slab-=cy(5.3,-9.1,9.1,0,[0,10.2,16])
 new=(old-region)+slab+fork
 env['add'](n,new,'carriage',bed);t=env['parts'][-1]['mesh'];t.export(O/(n+'.stl'));ss=solid(t)
 report.append(dict(part=n,removed_mm3=(old-ss).volume(),added_mm3=(ss-old).volume(),fork_removed_mm3=(fork-ss).volume()))
 for p in parts:
  if p['name']==n:p['s']=ss;p['mesh']=t
(O/'Changes.json').write_text(json.dumps(report,indent=2))
tail=source[source.index('printed_hits=[]'):].replace("(R/'Clearance checks.json')","(O/'Clearance checks.json')")
exec(compile(tail,'clearance checks','exec'))
assembly=(R0/'carriage-print-split/check_assembly.py').read_text().replace("O=R/'split-candidate'","O=R/'flat-face-candidate'");exec(compile(assembly,'assembly checks','exec'),{'__name__':'__main__','__file__':str(R0/'carriage-print-split/check_assembly.py')})
