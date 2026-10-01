from pathlib import Path
import runpy,os,json,numpy as np,trimesh,hashlib
R=Path(__file__).resolve().parents[1];O=R/'tile-candidate'
runpy.run_path(str(R/'tiling/check.py'),run_name='__main__')
runpy.run_path(str(R/'tiling/flush_access.py'),run_name='__main__')
d=json.loads((O/'Tiling checks.json').read_text());assert d['passed'],d['hits']
c=json.loads((O/'Clearance checks.json').read_text());assert not any(c[k] for k in ['printed_interferences','hardware_interferences','band_interferences'])
plate=[]
for name,count,x in [('Horizontal frame bridge',2,0),('Vertical frame bridge',2,32),('Centre frame bridge',1,56)]:
 for i in range(count):
  t=trimesh.load(O/(name+'.stl'));t.apply_transform(trimesh.geometry.align_vectors([0,-1,0],[0,0,-1]));t.apply_translation(-t.bounds[0]);t.apply_translation([x,30*i,0]);plate.append(t)
t=trimesh.util.concatenate(plate);t.export(O/'2x2 frame connectors print layout.stl')
(O/'Connector print layout.json').write_text(json.dumps(dict(bounds=t.bounds.tolist(),parts=5,bed_face='-Y',bores='vertical',layer_support='Constant cross-section up to the shallow exit counterbores; no support required by this geometry.'),indent=2))
# Render the actual 2x2 meshes using the existing offline three-view renderer.
s=(R/'render_review.py').read_text();a=s.index('idx=4;');b=s.index('tri=np.concatenate',a)
s=s[:a]+'''tri=[];cols=[]
for row in [0,1]:
 for col in [0,1]:
  idx=0 if row==0 else 26
  for p in D['parts']:
   b=D['bands'][idx].get(p['name'],p);a=v[b['offset']//3:b['offset']//3+b['vertices']].astype(float)
   if b is p:
    M=np.array(D['transforms'][idx][p['name']]).reshape(4,4).T;a=a@M[:3,:3].T+M[:3,3]
   a+=np.array([80*col,0,64*row]);a=a.reshape(-1,3,3);tri.append(a);cols.extend([p['color']]*len(a))
tv=np.frombuffer(gzip.decompress(base64.b64decode(D['tiling']['geometry'])),dtype='<f4').reshape(-1,3)
for p in D['tiling']['parts']:
 a=tv[p['offset']//3:p['offset']//3+p['vertices']].astype(float);f=D['frames'][0 if p['row']==0 else 26]
 if p['motion']=='carriage':a[:,0]+=f['q']
 if p['motion']=='lock':a[:,0]+=f['lock']
 a=a.reshape(-1,3,3);tri.append(a);cols.extend([p['color']]*len(a))
''' +s[b:]
s=s.replace("'Review views.png'","'2x2 review.png'").replace('Work in progress · bolt released · X: rods/axles; Z: rows; +Y: rear/base','2×2 · bases 160 mm X × 128 mm Z · rods coupled along X · +Y: rear/base')
exec(compile(s,'2x2 render','exec'),{'__name__':'__main__','__file__':str(R/'render_review.py')})
rear=s.replace("(1500,560)","(500,560)").replace("[('Front',0,0),('End',math.pi/2,0),('Oblique',-.4,.35)]","[('Rear / bottom',math.pi,0)]").replace("'2x2 review.png'","'Rear pin access.png'").replace('2×2 · bases 160 mm X × 128 mm Z · rods coupled along X · +Y: rear/base','Rear face Y = 48.4 mm · connecting pins flush')
exec(compile(rear,'rear render','exec'),{'__name__':'__main__','__file__':str(R/'render_review.py')})
(O/'Checked mesh hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.stl')},indent=2))
print('Tiling checks, connector print layout and actual-mesh views finished.',flush=True)
