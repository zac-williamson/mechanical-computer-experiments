"""Independent restart: only the unclocked planar register is a geometry source."""
from pathlib import Path
import re,json,gzip,base64,hashlib,shutil
import numpy as np,trimesh,manifold3d as m
ROOT=Path('/Users/zac/Documents/ChatGPT/lego designs 2_')
SOURCE=ROOT/'register-from-multiplexer/Planar register'
OUT=ROOT/'work/planar-module-restart/preview';OUT.mkdir(exist_ok=True)
s=(SOURCE/'Viewer.html').read_text();d=json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',s,re.S)[1])
v=np.frombuffer(gzip.decompress(base64.b64decode(d['geometry'])),dtype='<f4').reshape(-1,3)
meta=[];arrays=[];proof=[];solids={}
def box(a,b):return m.Manifold.cube((np.array(b)-a).tolist()).translate(a)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=False)
names={'Memory — Carriage fork and roof':'Carriage body — original planar register','Memory — Right carriage bearing support':'Carriage bearing end — original planar register','Direct lock bolt':'Locking bolt','Detachable bolt guide':'Locking bolt guide','Unified rear backbone':'Memory section of original frame','Memory — U015':'Actuator worm','Memory — U022':'Actuator reaction gear','Memory — L072':'Left clutch gear','Memory — L102':'Right clutch gear','Memory — L099':'Sliding clutch ring','Memory — L097':'Clutch axle connector'}
for p in d['parts']:
 n=p['id'];keep=p.get('bank')=='Memory' or n in ['Direct lock bolt','Detachable bolt guide','Unified rear backbone','Lock elastic band'] or n.startswith(('Cam roller','Guide mount friction pin','Frame pin Memory'))
 if not keep:continue
 # Routing gears belong to the device assembled from modules, not this core.
 if any(x in n for x in ['A-input','A-idler','B-input','common 1 power','inner bearing wall']):continue
 a=v[p['offset']//3:p['offset']//3+p['vertices']].copy()
 kind=p.get('kind');printed=kind in ['structure','printed']
 if printed:
  path=SOURCE/(n+'.stl');t=trimesh.load(path);h=hashlib.sha256(path.read_bytes()).hexdigest()
  if n=='Unified rear backbone':
   ss=solid(t)^box([-40,-100,-100],[60,100,100]);t=mesh(ss)
  else:ss=solid(t)
  assert t.is_watertight and len([q for q in ss.decompose() if q.volume()>1e-5])==1,n
  solids[n]=ss;a=t.triangles.reshape(-1,3)
  proof.append(dict(part=n,source=str(path.relative_to(ROOT)),sha256=h,change='Cropped away the neighbouring WRITE section at X = -40 mm' if n=='Unified rear backbone' else 'Unchanged source mesh'))
 col=[c/255 for c in p.get('color',[70,90,90])]
 item=dict(name=names.get(n,n.removeprefix('Memory — ')),source_name=n,motion=p['motion'],kind='printed' if printed else kind,color=col,offset=sum(x.size for x in arrays),vertices=len(a),bounds=[a.min(0).tolist(),a.max(0).tolist()],joint=p.get('joint'))
 meta.append(item);arrays.append(a)
# A diagnostic envelope, not an actual proposed rod or a printable component.
rod=mesh(box([-47.8,6.2,28.2],[47.8,14.2,35.8]));a=rod.triangles.reshape(-1,3)
meta.append(dict(name='Carriage rod at requested Z32 — interference study',source_name='study',motion='fixed',kind='study',color=[.85,.18,.17],offset=sum(x.size for x in arrays),vertices=len(a)));arrays.append(a)
report=dict(source=str(SOURCE),status='Original planar memory mechanism extracted; module ports and replacement frame still pending',carriage_printed_parts=2,unchanged_functional_parts=[p['part'] for p in proof if p['change']=='Unchanged source mesh'],provenance=proof,rod_study=json.loads((ROOT/'work/planar-module-restart/source-audit.json').read_text()),qualification='No new print release. Source mechanism and its known limitations are retained.')
D=dict(parts=meta,geometry=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode(),report=report)
(OUT/'Source provenance.json').write_text(json.dumps(report,indent=2));(OUT/'Model.json').write_text(json.dumps(D,separators=(',',':')))
(OUT/'Viewer.html').write_text((ROOT/'work/planar-module-restart/viewer.html').read_text().replace('__DATA__',json.dumps(D,separators=(',',':'))))
# A static view for inspection, generated under the same CPU limit.
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(11,8));ax=fig.add_subplot(111,projection='3d');offset=0
for p,a in zip(meta,arrays):
 if p['kind']=='study':continue
 tri=a.reshape(-1,3,3).copy()
 if p['motion'] in ['carriage','worm','clutch-ring']:tri[:,:,0]-=3.75
 # Plot X,Z,Y so Z is vertical in the image and +Y recedes.
 tri=tri[:,:,[0,1,2]]
 pc=Poly3DCollection(tri,facecolors=p['color'],edgecolors='none',linewidths=0);ax.add_collection3d(pc)
ax.set(xlim=(-52,52),ylim=(-8,45),zlim=(-26,80),xlabel='X',ylabel='Y',zlabel='Z');ax.set_box_aspect((104,53,106));ax.view_init(18,-68);ax.set_title('Planar register memory mechanism — source geometry')
fig.savefig(OUT/'Original mechanism.png',dpi=130);plt.close(fig)
print('Published independent source-core preview; two original carriage parts.',flush=True)
