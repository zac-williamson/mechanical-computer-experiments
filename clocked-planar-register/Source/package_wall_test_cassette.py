"""Small staged print pack: official orientations and the identical production parts."""
from pathlib import Path
import json,hashlib,shutil,zipfile,numpy as np,trimesh
from wall_print_geometry import qualify
O=Path(__file__).resolve().parents[1]/'Wall register';D=O/'Storage test cassette';S=D/'Print parts';S.mkdir(exist_ok=True)
h=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest();report=json.load(open(O/'Storage test cassette checks.json'));assert report['geometry_sha256']==h and report['cassette_pass']
assert json.load(open(O/'Storage module print checks.json'))['modular_print_pass']
P={p['id']:p for p in json.load(open(O/'parts.json'))};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
rows=[]
for n in report['print_parts']:
 if n=='Storage test base':source=D/'Storage test base - PRINT THIS.stl'
 else:
  sources=[O/d/(n+'.stl') for d in ['Print oriented storage modules','Print oriented bearing walls','Print oriented axle parts','Print oriented assembly parts']];source=next((p for p in sources if p.exists()),None)
  if source is None:
   assert n=='bit frame 0 removable fixture 4',n
   p=P[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);t,q=qualify(t,1,-1)
   # Unchanged production carrier: static overhangs were explicitly accepted
   # by the user. Record them; never include it in the new-part print pass.
   assert q['watertight'] and q['solids']==1,q
   q['scope']='Unchanged static actuator-cheek carrier; no axle bores or running surfaces. Overhangs remain; not included in the new support-free geometry claim.'
   source=D/(n+' - PRINT THIS.stl');t.export(source)
   (D/'Actuator carrier print orientation.json').write_text(json.dumps(q,indent=2))
 target=S/(n+'.stl');shutil.copyfile(source,target);t=trimesh.load(target)
 stage=1 if n in ['master rail and backing cartridge','master Carriage fork and roof','master Right carriage bearing support'] else 2 if n in ['Storage test base','master lock bolt','master bolt guide left','master bolt guide right','master replaceable band anchor'] else 3
 rows.append(dict(part=n,stage=stage,file=str(target.relative_to(D)),source=str(source.relative_to(O)),volume_mm3=float(t.volume),size_mm=t.extents.tolist(),sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
# Compact separate plates for each increment, never duplicate previous-stage parts.
layouts=[]
for stage in [1,2,3]:
 ts=[];places=[];x=y=row=0.
 for e in rows:
  if e['stage']!=stage:continue
  t=trimesh.load(D/e['file']);w,depth=t.extents[:2]
  if x+w>235:x=0;y+=row+5;row=0
  t.apply_translation([x,y,0]);ts.append(t);places.append(dict(part=e['part'],translation_mm=[x,y,0]));x+=w+5;row=max(row,depth)
 combined=trimesh.util.concatenate(ts);assert max(combined.extents[:2])<=245,combined.extents;file='Stage '+str(stage)+' - additional parts.stl';combined.export(D/file);layouts.append(dict(stage=stage,file=file,size_mm=combined.extents.tolist(),parts=places))
manifest=dict(geometry_sha256=h,parts=rows,layouts=layouts,total_printed_volume_mm3=sum(e['volume_mm3'] for e in rows),native_parts=[dict(part=n,lego_part=P[n].get('lego_part'),baseline_id=P[n].get('baseline_id'),kind=P[n]['kind']) for n in report['production_parts'] if P[n]['kind']!='printed'],limitations=['This is a hand-operated component test; no shared controller, sequencer, powered data path or bit logic test.','Reused production parts are unchanged; use the existing axle-bed-face report for those orientations. Strict complete-part overhang/layer checks apply to the new support pieces and base, not automatically to every inherited part.'])
(D/'Print manifest.json').write_text(json.dumps(manifest,indent=2))
shutil.copyfile(O/'Modular testing.md',D/'START HERE.md')
with zipfile.ZipFile(O/'Storage test cassette.zip','w',zipfile.ZIP_DEFLATED) as z:
 for f in D.rglob('*'):
  if f.is_file() and f.suffix not in ['.npz']:z.write(f,f.relative_to(D))
print('Test pack',len(rows),'printed parts; volume',manifest['total_printed_volume_mm3'],'mm3; no time estimate without slicing')
