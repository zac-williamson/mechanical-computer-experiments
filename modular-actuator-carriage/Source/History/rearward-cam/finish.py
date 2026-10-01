from pathlib import Path
import json,base64,gzip,hashlib
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'rearward-cam-candidate';B=O/'baseline'
assert json.loads((O/'Cam extension checks.json').read_text())['passed']
normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]}
checks=json.loads((B/'Bed orientation checks.json').read_text());placed={};reports=[]
for p in checks:
 name=p['part'];t=trimesh.load_mesh(O/(name+'.stl'))
 if name=='Locking bolt guide':p['bed_face']='bottom'
 t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed_face']],[0,0,-1]));t.apply_translation(-t.bounds[0])
 if name in ['Lock control rod']:
  assert t.is_watertight and len(t.split())==1;t.export(O/(name+' print.stl'),file_type='stl_ascii')
  sm=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)));islands=[]
  for h in np.arange(.3,t.extents[2],.2):
   sec=sm.slice(h);prev=sm.slice(h-.2)
   if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):islands.append(round(float(h),2))
  reports.append(dict(part=name,bed=p['bed_face'],islands=islands));assert not islands,(name,islands)
 t.apply_translation(p['bounds'][0]);p['bounds']=t.bounds.tolist();placed[name]=t
# Keep validated existing bed placements; the wider rod fits within its row gap.
for i,a in enumerate(checks):
 for b in checks[i+1:]:
  aa=np.array(a['bounds']);bb=np.array(b['bounds']);assert not np.all(np.minimum(aa[1,:2],bb[1,:2])-np.maximum(aa[0,:2],bb[0,:2])>.01),(a['part'],b['part'])
trimesh.util.concatenate(list(placed.values())).export(O/'Print layout.stl',file_type='stl_ascii')
pair=[];cursor=0
for n in ['Carriage body','Carriage bearing end']:
 t=placed[n].copy();t.apply_translation(-t.bounds[0]);t.apply_translation([cursor,0,0]);cursor+=t.extents[0]+8;pair.append(t)
trimesh.util.concatenate(pair).export(O/'Carriage print layout.stl',file_type='stl_ascii')
# Compact replacement set, in individually verified print orientations.
replacement=[];cursor=0
for n in ['Lock control rod']:
 t=trimesh.load_mesh(O/(n+' print.stl'));t.apply_translation([cursor,0,0]);cursor+=t.extents[0]+8;replacement.append(t)
trimesh.util.concatenate(replacement).export(O/'Lock rod print layout.stl',file_type='stl_ascii')
(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2));(O/'Cam print checks.json').write_text(json.dumps(reports,indent=2))
html=(B/'Viewer.html').read_text();tag='<script type="application/json" id="data">';pre,rest=html.split(tag,1);raw,post=rest.split('</script>',1);d=json.loads(raw);model=json.loads((O/'Model.json').read_text());d.update(model);d['print_centre']=trimesh.util.concatenate(list(placed.values())).bounds.mean(axis=0).tolist()
# Append all pose-dependent band meshes; their offsets are separate from Model.json.
source_html=(B/'Viewer.html').read_text();source=json.loads(source_html.split(tag,1)[1].split('</script>',1)[0]);sv=np.frombuffer(gzip.decompress(base64.b64decode(source['geometry'])),dtype='<f4').reshape(-1,3);base=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3);geo=[base];off=base.size
for i,frame in enumerate(d['bands']):
 for name,p in frame.items():
  src=source['bands'][i][name];a=sv[src['offset']//3:src['offset']//3+src['vertices']];p['offset']=off;p['vertices']=len(a);off+=a.size;geo.append(a)
d['geometry']=base64.b64encode(gzip.compress(np.concatenate(geo).astype('<f4').tobytes())).decode()
v=np.frombuffer(gzip.decompress(base64.b64decode(d['print_geometry'])),dtype='<f4').reshape(-1,3);arrays=[];offset=0
for p in d['print_parts']:
 a=placed[p['name']].triangles.reshape(-1,3).astype('<f4') if p['name'] in placed else v[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=offset;p['vertices']=len(a);offset+=a.size;arrays.append(a)
d['print_geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode();(O/'Viewer.html').write_text(pre+tag+json.dumps(d,separators=(',',':'))+'</script>'+post)
for n in ['Lock control rod print.stl','Lock rod print layout.stl','Print layout.stl']:
 t=trimesh.load_mesh(O/n);assert t.is_watertight,n
print('PRINT CHECKS',reports,flush=True)

full=trimesh.load_mesh(O/'Print layout.stl');(O/'Print layout checks.json').write_text(json.dumps(dict(parts=[p['part'] for p in checks],dimensions_mm=full.extents.tolist(),watertight=bool(full.is_watertight),solid_count=len(full.split())),indent=2))

p=O/'Viewer.html';h=p.read_text().replace('Whole-stud axles · 2L module connectors','Rearward cam support').replace('9L actuator axle · two 4L output axles · smooth 59443 connectors shown in 2×2 view.','Cam extended to Y = 19.6 mm · follower moved +4 mm along Y · original curved profile retained.');h=h.replace('<option value="worm-group">Carriage and worm</option>','<option value="cam-group">Lock rod, follower and bolt</option><option value="worm-group">Carriage and worm</option>').replace("const groups={'axle-group'","const groups={'cam-group':['Lock control rod','Locking bolt','Locking bolt guide','Bolt follower front bush','Bolt follower rear bush','Bolt follower axle 3L'],'axle-group'");p.write_text(h)
