from pathlib import Path
import json,numpy as np,trimesh,gzip,base64,re
R=Path(__file__).resolve().parents[1];B=R.parent/'work/register-mux-reference/multiplexer';O=R/'Latch coupon'
parts=[];arrays=[]
for n,mo,co in [('Front guide','fixed',[103,128,119]),('Rear guide','fixed',[103,128,119]),('Memory keeper','carriage',[36,151,165]),('Sliding bolt','bolt',[231,171,45])]:
 a=trimesh.load(O/(n+'.stl')).triangles.reshape(-1,3);parts.append(dict(id=n,motion=mo,kind='structure' if mo=='fixed' else 'printed',color=co,offset=sum(x.size for x in arrays),vertices=len(a)));arrays.append(a)
hm=json.loads((B/'hardware.json').read_text());v=np.load(B/'hardware.npz')['vertices'];p=next(x for x in hm if x['id'].startswith('Baseboard pin'));a=v[p['offset']//3:p['offset']//3+p['vertices']].copy();a-=(a.min(0)+a.max(0))/2
for x in [-30,30]:
 b=a+[x,2.4,6];parts.append(dict(id=f'LEGO 2780 at X{x}',motion='fixed',kind='lego',color=[57,83,107],offset=sum(x.size for x in arrays),vertices=len(b)));arrays.append(b)
trace=[]
def segment(q0,q1,s0,s1,label):
 for t in np.linspace(0,1,81):trace.append(dict(q=q0+(q1-q0)*float(t),s=s0+(s1-s0)*float(t),b=0,w=0,g=0,segment=label))
segment(-4.3,4.3,-10,-10,'Unlocked: carriage traverses')
segment(4.3,4.3,-10,0,'Bolt inserts into endpoint pocket')
segment(4.3,4.3,0,0,'Locked: endpoint retained')
segment(4.3,4.3,0,-10,'Bolt withdraws before movement')
segment(4.3,-4.3,-10,-10,'Unlocked: carriage traverses')
segment(-4.3,-4.3,-10,0,'Bolt inserts into other endpoint pocket')
segment(-4.3,-4.3,0,0,'Locked: other endpoint retained')
segment(-4.3,-4.3,0,-10,'Bolt withdraws')
D=dict(parts=parts,bounds=[[-34,-30,-4],[34,10.4,16]],pivot=[0,0,0],reaction=[0,0,0],trace=trace,geometry=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode())
tpl=(B/'Source/viewer.html.in').read_text();js=tpl[tpl.index('<script>'):]
js=js.replace("if(mo==='carriage')t=[f.q,0,0];","if(mo==='carriage')t=[f.q,0,0];if(mo==='bolt')t=[0,f.s,0];")
start=js.index("$('#status').textContent=");end=js.index(';if(playing)',start)
js=js[:start]+"$('#status').textContent=f.segment+' · Keeper '+f.q.toFixed(2)+' mm · Bolt '+f.s.toFixed(2)+' mm'"+js[end:]
js=js.replace('Play switching','Play coupon motion').replace("let az=-1.1", "let az=-1.1")
html='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Latch coupon — fit study</title><style>body{margin:0;background:#f6f5ee;color:#263e40;font:15px/1.5 system-ui}main{max-width:1200px;margin:auto;padding:22px}h1{margin:4px 0;font-size:28px}.note{padding:12px;background:#eee3c7}.tools{display:flex;flex-wrap:wrap;gap:14px;margin:12px 0;align-items:center}button,select{padding:7px;border:1px solid #87998d;background:#fff;border-radius:5px;color:inherit}canvas{width:100%;height:62vh;min-height:380px;touch-action:none}#status{min-height:24px}a{color:#176a75}</style><main><a href="../Report.html">← Register investigation</a><h1>Captive sliding latch · test coupon</h1><p>Teal: memory keeper. Gold: bolt. Grey-green: two pin-connected guides.</p><p class="note">This is a four-part fit/load test fixture, not an assembled 1-bit register. Motion is prescribed; no forces or friction are simulated. Actuator, cam, rubber-band lost motion and signal routes are not fitted.</p><div class="tools"><button id="run">Play coupon motion</button><button id="xz">XZ view</button><button id="iso">Perspective view</button><button id="zy">Side view</button><button id="fit">Fit</button><label><input id="frame" type="checkbox" checked>Guide structure</label><label>Inspect <select id="part"><option value="">Complete coupon</option><option value="carriage">Keeper only</option></select></label></div><div style="display:none"><select id="a"><option value="-1">0</option></select><select id="b"><option value="1">1</option></select></div><input id="timeline" type="range" min="0" max="647" value="0" style="width:100%"><div id="status"></div><div style="position:relative"><canvas id="view"></canvas><svg id="axes" viewBox="0 0 190 190" width="190" height="190" aria-label="Model coordinate axes" style="position:absolute;right:8px;top:8px;pointer-events:none;background:#ffffffdd;border-radius:8px"></svg></div><p>Drag to rotate · Shift-drag to pan · Scroll to zoom. Turn off guide structure to inspect the pockets.</p><p><a href="README.md">Assembly and test procedure</a> · <a href="Print%20layout.stl">Print layout</a> · <a href="Checks.json">Geometry and analytical checks</a></p><div id="error"></div></main>'''
(O/'Viewer.html').write_text(html+'<script type="application/json" id="data">'+json.dumps(D,separators=(',',':'))+'</script>'+js)
# Layerwise geometric support screen, using actual bed-oriented exported files.
from shapely.geometry import Polygon
from shapely.ops import unary_union
support=[]
for f in (O/'Print parts').glob('*.stl'):
 t=trimesh.load(f);previous=None;bad=[]
 for z in np.arange(.1,t.bounds[1,2],.2):
  cut=t.section(plane_origin=[0,0,float(z)],plane_normal=[0,0,1]);shape=Polygon()
  if cut is not None:
   for loop in cut.discrete:
    pg=Polygon(loop[:,:2]);shape=shape.symmetric_difference(pg)
  if previous is not None:
   a=shape.difference(previous.buffer(.2001)).area
   if a>.01:bad.append(dict(z=float(z),area_mm2=float(a)))
  previous=shape
 support.append(dict(part=f.stem,layer_mm=.2,maximum_overhang_horizontal_per_layer_mm=.2,unsupported_growth=bad))
(O/'Layer support screen.json').write_text(json.dumps({'method':'Each 0.2mm slice must lie within previous slice expanded 0.2mm. Geometric 45-degree growth screen, not a slicer validation.','parts':support},indent=2))
print('Published coupon viewer and layer support screen:',{x['part']:len(x['unsupported_growth']) for x in support})
