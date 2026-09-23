from pathlib import Path
import json,gzip,base64,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];B=R.parent/'work/register-mux-reference/multiplexer';O=R/'Linkage development'
meta=json.loads((O/'printed-parts.json').read_text());hm=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];parts=[];arrays=[]
for p in meta:
 a=trimesh.load(O/(p['id']+'.stl')).triangles.reshape(-1,3);color=[134,149,134] if p['motion']=='fixed' else [29,147,157] if p['bank']=='Memory' else [190,117,71] if p['bank']=='Write' else [211,160,46]
 parts.append(dict(**p,kind='structure' if p['motion']=='fixed' else 'printed',color=color,offset=sum(x.size for x in arrays),vertices=len(a)));arrays.append(a)
for p in hm:
 a=v[p['offset']//3:p['offset']//3+p['vertices']];pp=dict(p);pp.update(color=p.get('color',[66,79,91]),offset=sum(x.size for x in arrays));parts.append(pp);arrays.append(a)
frames=json.loads((B/'Switching trace.json').read_text())['frames']
def beta(q):return min(frames,key=lambda f:abs(f['q']-q))['b']
trace=[]
def segment(m0,m1,e0,e1,label):
 for t in np.linspace(0,1,100):
  qm=m0+(m1-m0)*float(t);qe=e0+(e1-e0)*float(t);cmd=-15+20*float(np.clip((qe-.9)/1.5,0,1));trace.append(dict(q=qm,qm=qm,qe=qe,s=min(0,cmd),angle=float(np.arcsin((5.8+cmd)/50)),bm=beta(qm),be=beta(qe),b=0,w=0,g=0,segment=label))
segment(-4.3,-4.3,4.3,-4.3,'WRITE begins: withdraw lock, then reconnect D')
segment(-4.3,4.3,-4.3,-4.3,'D = 1: memory carriage changes state')
segment(4.3,4.3,-4.3,4.3,'HOLD begins: disconnect D, then insert lock')
segment(4.3,4.3,4.3,4.3,'HOLD: Q remains 1; D is disconnected')
segment(4.3,4.3,4.3,-4.3,'WRITE begins: withdraw lock, then reconnect D')
segment(4.3,-4.3,-4.3,-4.3,'D = 0: memory carriage changes state')
segment(-4.3,-4.3,-4.3,4.3,'HOLD begins: disconnect D, then insert lock')
allv=np.concatenate(arrays);bounds=[(allv.min(0)-[5,0,16]).tolist(),(allv.max(0)+[5,0,5]).tolist()]
D=dict(parts=parts,bounds=bounds,pivot=[0,0,0],reaction=[0,0,0],trace=trace,geometry=base64.b64encode(gzip.compress(allv.astype('<f4').tobytes())).decode())
tpl=(B/'Source/viewer.html.in').read_text();js=tpl[tpl.index('<script>'):]
start=js.index("if(mo==='carriage')");end=js.index('gl.uniformMatrix3fv(L.model',start)
js=js[:start]+'''let q=p.bank==='Memory'?f.qm:f.qe,bankSign=p.bank==='Write'?-1:1;
if(mo==='carriage'||mo==='worm')t=[bankSign*q,0,0];
if(mo==='write-cam')t=[-f.qe,0,0];
if(mo==='bolt')t=[0,0,f.s];
if(mo==='clutch-ring')t=[bankSign*Math.sign(q)*Math.max(Math.abs(q)-.4,0),0,0];
if(mo==='rocker'&&p.bank==='Linkage')[R,t]=rot('Y',f.angle,[50,0,2.7]);
if(mo==='rocker'&&p.bank==='Memory')[R,t]=rot('Y',f.bm*Math.PI/180,[13.192323604,10.2,32.128448698]);
if(mo==='rocker'&&p.bank==='Write')[R,t]=rot('Y',f.be*Math.PI/180,[83.607676396,10.2,-16.128448698]);
''' +js[end:]
start=js.index("$('#status').textContent=");end=js.index(';if(playing)',start);js=js[:start]+"$('#status').textContent=f.segment+' · Memory '+f.qm.toFixed(1)+' mm · Write '+f.qe.toFixed(1)+' mm'"+js[end:]
js=js.replace('gl.uniform3f(L.center,0,15,8)','gl.uniform3f(L.center,48,35,8)').replace('D.bounds[k&1?1:0][0],D.bounds[k&2?1:0][1]-15','D.bounds[k&1?1:0][0]-48,D.bounds[k&2?1:0][1]-35').replace('Play switching','Play sequence')
head='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Mechanical register · development assembly</title><style>body{margin:0;background:#f6f5ee;color:#263e40;font:15px/1.5 system-ui}main{max-width:1400px;margin:auto;padding:22px}h1{margin:4px 0;font-size:28px}.note{padding:12px;background:#eee3c7}.tools{display:flex;flex-wrap:wrap;gap:14px;margin:12px 0;align-items:center}button,select{padding:7px;border:1px solid #87998d;background:#fff;border-radius:5px;color:inherit}canvas{width:100%;height:65vh;min-height:380px;touch-action:none}#status{min-height:24px}a{color:#176a75}</style><main><a href="../Report.html">← Design investigation</a><h1>Write-disconnected 1-bit register</h1><p>Teal: memory actuator. Copper: write/disconnect actuator. Gold: lock linkage. Grey-green: printed supports.</p><p class="note">Development geometry, not a print release or a proven 0.2 Nm mechanism. Animation prescribes the sequence; it does not simulate dog-clutch alignment, friction, band forces or inertia. Elastic band route is not rendered. Read the report before using these parts.</p><div class="tools"><button id="run">Play sequence</button><button id="xz">XZ view</button><button id="iso">Perspective view</button><button id="zy">Side view</button><button id="fit">Fit</button><label><input id="frame" type="checkbox" checked>Support structure</label><label>Inspect <select id="part"><option value="">Complete assembly</option><option value="carriage">Moving carriages</option></select></label></div><div hidden><select id="a"><option value="1">1</option></select><select id="b"><option value="-1">0</option></select></div><input id="timeline" type="range" min="0" max="699" value="0" style="width:100%"><div id="status"></div><div style="position:relative"><canvas id="view"></canvas><svg id="axes" viewBox="0 0 190 190" width="190" height="190" style="position:absolute;right:8px;top:8px;pointer-events:none;background:#ffffffdd;border-radius:8px"></svg></div><p>Drag to rotate · Shift-drag to pan · Scroll to zoom. The source multiplexer lever pose is approximated from its contact trace; displayed shaft rotation is omitted.</p><div id="error"></div></main>'''
(O/'Viewer.html').write_text(head+'<script type="application/json" id="data">'+json.dumps(D,separators=(',',':'))+'</script>'+js)
# True nominal bounds, separately from padded viewer framing; motion envelope handled by checks.
(O/'Envelope.json').write_text(json.dumps(dict(nominal_bounds_mm=[allv.min(0).tolist(),allv.max(0).tolist()],nominal_size_mm=np.ptp(allv,axis=0).tolist(),qualification='Zero-angle reference geometry. Not a swept envelope or a proven minimum.'),indent=2))
print('Published self-contained assembly viewer')
