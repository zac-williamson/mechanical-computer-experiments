from pathlib import Path
import sys,shutil,json,base64,gzip
import numpy as np,trimesh
R=Path(__file__).resolve().parents[1];A=R/'neck-candidate';O=R/'axle-tiling-candidate';B=O/'baseline';B.mkdir(parents=True,exist_ok=True)
for p in A.iterdir():
 if p.is_file() and p.suffix in ['.stl','.json','.md','.html']:
  if not (B/p.name).exists():shutil.copy2(p,B/p.name)
  shutil.copy2(B/p.name,O/p.name)
sys.path.insert(0,str(R.parents[1]/'latest-register-analysis/planar-register/register-from-multiplexer/Source'))
from ldraw_mesh import LDraw
lib=LDraw('/Applications/Studio 2.0/ldraw/parts/60485.dat');a=lib.mesh()[0].reshape(-1,3)*.4;assert not lib.missing
ax=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);ax.apply_translation(-ax.bounds.mean(0));axis=int(np.argmax(ax.extents));ax.apply_transform(trimesh.geometry.align_vectors(np.eye(3)[axis],[1,0,0]));ax.apply_translation([0,10.2,16]);assert abs(ax.extents[0]-72)<1e-5
D=json.loads((B/'Model.json').read_text());tag='<script type="application/json" id="data">';html=(B/'Viewer.html').read_text();pre,rest=html.split(tag,1);raw,post=rest.split('</script>',1);V=json.loads(raw)
def unpack(d,key='geometry'):return np.frombuffer(gzip.decompress(base64.b64decode(d[key])),dtype='<f4').reshape(-1,3)
def pack(a):return base64.b64encode(gzip.compress(np.concatenate(a).astype('<f4').tobytes())).decode()
original=unpack(D);meshes={};report=[]
for p in D['parts']:
 a=original[p['offset']//3:p['offset']//3+p['vertices']].copy()
 if p['name']=='C-shaft':a=ax.triangles.reshape(-1,3);p['axis']=0
 elif p['name']=='Left output axle 4L':a[:,0]-=1.8
 elif p['name']=='Right output axle 4L':a[:,0]+=1.8
 else:continue
 t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);meshes[p['name']]=t;report.append(dict(part=p['name'],bounds=t.bounds.tolist(),length_mm=float(t.extents[0]),studs=round(float(t.extents[0])/8)))
arr=[];off=0
for p in D['parts']:
 a=meshes[p['name']].triangles.reshape(-1,3) if p['name'] in meshes else original[p['offset']//3:p['offset']//3+p['vertices']]
 p['offset']=off;p['vertices']=len(a);off+=a.size;arr.append(a)
D['geometry']=pack(arr);(O/'Model.json').write_text(json.dumps(D,separators=(',',':')))
oldv=unpack(V);oldbands=V['bands'];V.update(D)
for frame in oldbands:
 for b in frame.values():
  a=oldv[b['offset']//3:b['offset']//3+b['vertices']];b['offset']=off;off+=a.size;arr.append(a)
V['geometry']=pack(arr);V['labels']['C-shaft']='Actuator input axle 9L'
T=V['tiling'];ta=[unpack(T)];toff=ta[0].size;connector=trimesh.load_mesh(R/'axle-tiling/59443.stl')
for row in [0,1]:
 for z,label in [(0,'Output'),(16,'Actuator')]:
  t=connector.copy();t.apply_translation([40,10.2,z+64*row]);a=t.triangles.reshape(-1,3);name=f'{label} 2L axle connector, row {row+1}'
  T['parts'].append(dict(name=name,offset=toff,vertices=len(a),kind='native',motion='fixed',row=row,color=[.53,.55,.57],axis=0));toff+=a.size;ta.append(a)
T['geometry']=pack(ta);(O/'Tiling geometry.json').write_text(json.dumps(T,separators=(',',':')))
pre=pre.replace('Reinforced guide · single return band','Whole-stud axles · 2L module connectors').replace('Thicker guide walls, narrower rounded bolt and one continuous return band.','9L actuator axle · two 4L output axles · smooth 59443 connectors shown in 2×2 view.')
pre=pre.replace('<option value="all">Whole module</option>','<option value="all">Whole module</option>')
post=post.replace('<option value="worm-group">Carriage and worm</option>','<option value="axle-group">Axles and module connectors</option><option value="worm-group">Carriage and worm</option>')
post=post.replace("const groups={'lock-group'","const groups={'axle-group':['C-shaft','Left output axle 4L','Right output axle 4L','L097'],'lock-group'")
post=post.replace("if(mode==='tiles'&&$('part').value==='all')for(const p of D.tiling.parts){if(!$('frame').checked&&p.motion==='fixed')continue;", "if(mode==='tiles'&&['all','axle-group'].includes($('part').value))for(const p of D.tiling.parts){const axleConnector=p.name.includes('2L axle connector');if($('part').value==='axle-group'&&!axleConnector)continue;if(!$('frame').checked&&p.motion==='fixed'&&!axleConnector)continue;")
(O/'Viewer.html').write_text(pre+tag+json.dumps(V,separators=(',',':'))+'</script>'+post)
(O/'Axle dimensions.json').write_text(json.dumps(dict(pitch_X_mm=80,connector='59443 smooth 2L',connector_extent_X=[32,48],axles=report,input_engagement_mm=4,output_engagement_mm=6,internal_output_engagement_mm=6),indent=2));print(report,flush=True)
