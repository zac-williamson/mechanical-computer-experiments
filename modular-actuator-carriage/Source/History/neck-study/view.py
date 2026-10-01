from pathlib import Path
import json,gzip,base64
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];A=R/'simplified-carriage-r1';O=R/'neck-candidate'
h=(A/'Viewer.html').read_text();start='<script type="application/json" id="data">';ds=h.split(start)[1].split('</script>')[0];D=json.loads(ds);v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arrays=[v];off=v.size;D['before']={};D['reinforcement']={}
def append(t,name):
 global off
 a=t.triangles.reshape(-1,3);p=dict(name=name,offset=off,vertices=len(a));off+=a.size;arrays.append(a);return p
for p in D['parts']:
 if p['name'] not in ['Carriage body','Carriage bearing end']:continue
 n=p['name'];D['before'][n]=dict(p)
 new=trimesh.load(O/(n+'.stl'));old=trimesh.load(A/(n+'.stl'))
 def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 added=solid(new)-solid(old);mm=added.to_mesh64();delta=trimesh.Trimesh(mm.vert_properties[:,:3],mm.tri_verts,process=False)
 D['reinforcement'][n]=append(delta,n);npiece=append(new,n);p.update(offset=npiece['offset'],vertices=npiece['vertices'])
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode();h=h.replace(ds,json.dumps(D,separators=(',',':')))
h=h.replace('<h1>Carriage simplification — revision 1</h1>','<h1>Bearing connection — reinforcement comparison</h1>')
h=h.replace('<div id="stage">','<div class="tools"><label>Compare <select id="comparison"><option value="highlight">Reinforced — added material orange</option><option value="after">Reinforced — normal colours</option><option value="before">Before reinforcement</option></select></label><button id="inspect-neck">Inspect bearing connection</button></div><div id="stage">')
h=h.replace('buffers(D.parts,v);','buffers(Object.values(D.before),v);buffers(Object.values(D.reinforcement),v);buffers(D.parts,v);')
h=h.replace("const actual=mode!=='print'&&D.bands[pose][p.name]||p;", "const choice=$('comparison').value;const actual=mode!=='print'&&D.before[p.name]&&choice!=='after'?D.before[p.name]:(mode!=='print'&&D.bands[pose][p.name]||p);")
h=h.replace('render(p,actual,matrix);\n }',"render(p,actual,matrix);if(mode!=='print'&&D.reinforcement[p.name]&&choice==='highlight')render({...p,color:[1,.38,.03]},D.reinforcement[p.name],matrix);\n }")
h=h.replace("window.onresize=draw;", "$('comparison').oninput=draw;$('inspect-neck').onclick=()=>{setmode('assembly');$('frame').checked=false;$('part').value='Carriage body';zoom=2;az=-.7;el=.65;draw()};window.onresize=draw;")
h=h.replace("setmode('assembly');window.modelViewer", "setmode('assembly');$('frame').checked=false;$('comparison').value='highlight';draw();window.modelViewer")
h=h.replace('<button id="print">','<button hidden id="print">')
h=h.replace('<footer>','<footer><p><strong>Orange is the actual added material.</strong> Switch between before and reinforced at the same camera angle. Candidate for inspection: print layout is not updated or qualified.</p>')
# Do not offer the previous revision print files from the candidate.
import re
h=re.sub(r'<p><a href="Print%20layout.stl">.*?</p>','',h)
(O/'Viewer.html').write_text(h)
print('Comparison viewer saved',flush=True)
