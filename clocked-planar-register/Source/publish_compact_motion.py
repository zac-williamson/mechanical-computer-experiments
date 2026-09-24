"""Publish the compact angle-driven inspection, sharing audit transforms."""
from pathlib import Path
import json,gzip,base64,re
import numpy as np
from compact_pose import joint,vertices
R=Path(__file__).resolve().parents[1];O=R/'Compact layout'
p=json.loads((O/'parts.json').read_text());v=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
report=json.loads((O/'Compact contact-resolved operation.json').read_text());
import hashlib
assert report.get('contact_pose_pass'), 'Unresolved actuator contacts'
assert report['geometry_sha256']==hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(), 'Stale contact trace'
frames=next(c['frames'] for c in report['cases'] if c['start']==[1,1,0] and c['end']==[1,1,1] and c['initial_Q']==0)
delta=np.array([48,18,62])-(v.min(0)+v.max(0))/2
s=(R/'Compact layout.html').read_text();a=s.index('<script id="data" type="application/json">')+len('<script id="data" type="application/json">');b=s.index('</script>',a);data=json.loads(s[a:b])
out=[];elastic=[]
for f in frames:
 ff={k:f[k] for k in ['turns','Q','master','slave']};ff['joints']=[]
 for part in p:
  sh,ax,cen,ang=joint(part,f);ff['joints'].append([sh.tolist(),ax.tolist(),(cen+delta).tolist(),ang])
 out.append(ff)
# Elastic geometry is independently regenerated/deformed at every recorded pose.
indices=[i for i,part in enumerate(p) if part['kind']=='elastic']
for f in frames:
 for i in indices:
  part=p[i];a0=v[part['offset']//3:part['offset']//3+part['vertices']]
  elastic.append((vertices(part,a0,f)+delta).astype('<f4').ravel())
data['frames']=out;data['elasticParts']=indices;data['elasticGeometry']=base64.b64encode(gzip.compress(np.concatenate(elastic).tobytes())).decode()
s=s[:a]+json.dumps(data,separators=(',',':'))+s[b:]
s=s.replace('<button hidden id="play">','<button id="play">').replace('<input hidden aria-label="Capture progress"','<input aria-label="Capture progress"')
s=s.replace('Static placement study — no operation animation',"Angle-driven inspection — not mechanically qualified")
s=s.replace('Geometry is shown at fixed reference angles.','The animation uses the same transforms as the printed-motion audit; tooth phasing, forces and elastic preload remain unqualified.')
s=s.replace('`Angle-driven inspection — not mechanically qualified`',"`Input turns ${f.turns.toFixed(2)} · Q ${f.Q===null?'undriven':f.Q} · Master ${f.master.mode} · Output ${f.slave.mode} · Unqualified inspection`")
needle="let az=0,el=0,zoom=1,playing=false,last=0,index=0;"
if 'const ez=Uint8Array.from(atob(D.elasticGeometry)' in s:
 start=s.index('const ez=Uint8Array.from(atob(D.elasticGeometry)')
 finish=s.index(needle,start)
 s=s[:start]+s[finish:]
js="""const ez=Uint8Array.from(atob(D.elasticGeometry),c=>c.charCodeAt(0));
const ev=new Float32Array(await new Response(new Blob([ez]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
const elasticStride=D.elasticParts.reduce((n,i)=>n+parts[i].vertices*3,0);let elasticFrame=-1;
function updateElastic(index){if(index===elasticFrame)return;let offset=index*elasticStride;
for(const pi of D.elasticParts){const p=parts[pi],a=ev.subarray(offset,offset+p.vertices*3);offset+=p.vertices*3;let out=new Float32Array(a.length*2);
for(let i=0;i<a.length;i+=9){let x=a[i+3]-a[i],y=a[i+4]-a[i+1],z=a[i+5]-a[i+2],u=a[i+6]-a[i],vv=a[i+7]-a[i+1],w=a[i+8]-a[i+2],nx=y*w-z*vv,ny=z*u-x*w,nz=x*vv-y*u,l=Math.hypot(nx,ny,nz)||1;for(let j=0;j<9;j+=3)out.set([a[i+j],a[i+j+1],a[i+j+2],nx/l,ny/l,nz/l],(i+j)*2)}
gl.bindBuffer(gl.ARRAY_BUFFER,p.buffer);gl.bufferData(gl.ARRAY_BUFFER,out,gl.DYNAMIC_DRAW)}elasticFrame=index;}
"""
assert needle in s;s=s.replace(needle,js+needle).replace('function draw(){const ratio','function draw(){updateElastic(index);const ratio')
metrics=json.loads((O/'Layout metrics.json').read_text());width,depth=metrics['proposed_frame_envelope_xz_mm']
s=s.replace('Compact register · component layout','Compact register · mechanical model')
note=f'Envelope {width:.1f} × {depth:.1f} mm; {metrics["proposed_area_reduction_percent"]:.1f}% less XZ area; Y depth {metrics["placed_geometry_mm"][1]:.1f} mm. Animation includes clutch tooth pickup, backlash and resolved actuator contacts. Loaded operation and the inherited worm meshes remain unresolved. <a href="Compact%20layout/Validation%20status.md">Validation evidence</a>.'
s=re.sub(r'<p class="note">.*?</p>',lambda _: '<p class="note">'+note+'</p>',s,count=1,flags=re.S)
(R/'Compact layout.html').write_text(s)
print('Published',len(frames),'shared-transform frames and',len(indices),'deforming bands')
