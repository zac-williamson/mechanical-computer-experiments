"""Publish single-bit and vertical-bank inspection from the same geometry."""
from pathlib import Path
import json,gzip,base64,re
import numpy as np
from wall_pose import joint,vertices,example_frames,PHASES
R=Path(__file__).resolve().parents[1];O=R/'Wall register'
assert PHASES, 'Generate passing, current gear phases before publishing'
p=json.loads((O/'parts.json').read_text());v=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
for part in p:
 if 'removable bearing wall' in part['id'] or 'removable fixture' in part['id']:part['color']=[.42,.65,.60]
 elif 'coordinated chassis' in part['id']:part['color']=[.25,.43,.41]
frames=example_frames()[::5];delta=np.array([48,18,62])-(v.min(0)+v.max(0))/2
ff=[]
for f in frames:
 rec=dict(turns=f['turns'],Q=f['Q'],master=f['master'],slave=f['slave'],joints=[])
 for part in p:
  sh,ax,c,an=joint(part,f);rec['joints'].append([[0,0,0],[1,0,0],[0,0,0],0] if part['kind']=='elastic' else [sh.tolist(),ax.tolist(),(c+delta).tolist(),an])
 ff.append(rec)
specs=[
 ('A','WRITE actuator','Control write U015','control','left'),
 ('B','CLOCK actuator','Control clock U015','control','right'),
 ('C','CLOCK stroke amplifier','Control Clock amplifier front and input shoe','control','right'),
 ('D','Rotated CLOCK input','Control CLK external axle','control','left'),
 ('E','Rotated WRITE input','Control WRITE input axle','control','left'),
 ('F','Shared CLOCK rod','Control CLOCK direct rod and pickup','control','right'),
 ('G','Shared WRITE rod','Control WRITE direct rod and pickup','control','left'),
 ('H','WRITE selector','write L099','bit','left'),
 ('I','Master storage','master U015','bit','left'),
 ('J','Slave / output storage','slave U015','bit','right'),
 ('K','Local clock sequencing bar','Local clock cam and fork bar','bit','right'),
 ('L','Rotational output Q','slave output right 7L','bit','right'),
 ('M','Local POWER input','POWER local input 4L','bit','left'),
 ('N','Shared +POWER shaft','Shared reverse shaft left 10L','bit','right'),
 ('O','Shared −POWER shaft','POWER distribution 12L','bit','right'),
]
annotations=[]
for tag,label,part_id,scope,side in specs:
 i=next(i for i,part in enumerate(p) if part['id']==part_id)
 mesh=v[p[i]['offset']//3:p[i]['offset']//3+p[i]['vertices']]
 target=np.mean(p[i]['bounds'],axis=0)
 if tag=='F':target=np.array([-116.,10.,-72.])
 if tag=='G':target=np.array([-128.,26.,-88.])
 point=mesh[np.argmin(np.linalg.norm(mesh-target,axis=1))]+delta
 annotations.append(dict(tag=tag,label=label,part=i,point=point.tolist(),scope=scope,side=side))
data=dict(annotations=annotations,parts=p,frames=ff,geometry=base64.b64encode(gzip.compress((v+delta).astype('<f4').tobytes())).decode(),ports=[])
elastic=[]
for i,part in enumerate(p):
 if part['kind']!='elastic':continue
 a=v[part['offset']//3:part['offset']//3+part['vertices']]
 animated=np.concatenate([vertices(part,a,f)+delta for f in frames]).astype('<f4')
 elastic.append(dict(part=i,geometry=base64.b64encode(gzip.compress(animated.tobytes())).decode()))
data['elastic']=elastic
s=(R/'Source/assembly.html').read_text().replace('__DATA__',json.dumps(data,separators=(',',':')))
s=s.replace('Clocked planar register — development assembly','Modular wall register').replace('Clocked planar register · full development assembly','Modular wall register · shared controls')
s=re.sub(r'<p class="note">.*?</p>','<p class="note">Development CAD. One removable control module serves 1–8 planar rows. Light teal: removable bearing walls and working fixtures; dark teal: frame; orange: controls. Equal-tooth gear routes preserve engaged speed magnitude. <strong>Not a print release: see the development checks and design notes.</strong></p>',s,count=1)
s=s.replace('<button id="play">','<label>Rows <select id="rows"><option>1</option><option>2</option><option>4</option><option>8</option></select></label><button id="play">')
s=s.replace('<p><a href="README.md">Design status</a> · <a href="Layout%20fixed-part%20checks.json">Fixed-part audit</a> · <a href="Angle-driven%20operation.json">Operation report</a></p>','<p><a href="Wall%20register/Design.md">Design and assembly notes</a> · <a href="Wall%20register/Bearing%20bracing%20revision.md">Bearing bracing changes</a> · <a href="Wall%20register/Development%20checks.json">Development checks</a> · <a href="Compact%20layout.html">Previous compact model</a></p>')
s=s.replace('let az=.12,el=.35,zoom=1,playing=false,last=0,index=0;','let az=0,el=0,zoom=1,playing=false,last=0,index=0,rows=1;')
s=s.replace('const sx=zoom/235,sy=sx*canvas.width/canvas.height;', 'const sx=zoom/Math.max(175,(170+(rows-1)*56)*canvas.width/canvas.height),sy=sx*canvas.width/canvas.height;')
s=s.replace("for(let i=0;i<parts.length;i++){const p=parts[i],[shift,axis,origin,spin]=f.joints[i];", "for(let row=0;row<rows;row++){for(let i=0;i<parts.length;i++){const p=parts[i];if(row>0&&p.module==='control')continue;const [baseShift,axis,baseOrigin,spin]=f.joints[i];const offset=(p.module==='control'?0:row*112)-(rows-1)*56;const shift=[...baseShift];shift[2]+=offset;const origin=baseOrigin;")
s=s.replace('gl.drawArrays(gl.TRIANGLES,0,p.vertices)}ctx.clearRect','gl.drawArrays(gl.TRIANGLES,0,p.vertices)}}ctx.clearRect')
s=s.replace('window.onresize=draw;',"document.querySelector('#rows').onchange=e=>{rows=+e.target.value;draw()};window.onresize=draw;")
s=s.replace('`Input turns ${f.turns.toFixed(2)} · Q ${f.Q===null?\'undriven\':f.Q} · Master ${f.master.mode} · Output ${f.slave.mode}`','`${rows} row${rows>1?"s":""} · illustrative shared capture · Q ${f.Q===null?"disconnected":f.Q} · development geometry`')
s=s.replace('Example: D=1, WRITE=1, CLK rises; Q starts at 0.', 'Illustrative capture: D=1, WRITE=1, CLK rises; Q starts at 0. Repeated rows show the same example, not independent data simulation. Elastic loops are shown in reference shape.')
# Labels are tied to part transforms, so they follow the animation and orbit.
s=s.replace('<button id="play">','<label>Labels <select id="labelMode"><option value="all">All</option><option value="control">Shared controls</option><option value="bit">One bit</option><option value="off">Off</option></select></label><button id="play">')
s=s.replace('<strong>Not a print release:', '<strong>Frame revision: flat-bed frame parts with 45° joint-hole roofs; guide fixtures with LEGO friction pins (at least two each); pinned rod splice plates. Bearing walls retain two mounting pins each. Row pitch 112 mm.</strong> <strong>Not a print release:')
s=s.replace('</style>', ' .explainer{max-width:1050px;line-height:1.55}.explainer dt{font-weight:650;margin-top:12px}.explainer dd{margin:3px 0 12px}#labelMode{max-width:160px}</style>')
explanation="""<section class="explainer"><h2>Direct shared controls</h2>
<p>The controller uses two complete actuators rotated through 90°, including their input axles. Their outputs now travel along the vertical rods. The controller's direction-changing bellcranks and long horizontal clock link have been removed.</p>
<dl><dt>E → A → G: WRITE</dt><dd>The rotated WRITE input (E) drives actuator A. Its carriage is joined to the shared WRITE rod G, without a direction-changing lever. Rod travel is now ±3.75 mm. Each bit retains an equal-arm bellcrank with free roller followers to move its selector H. WRITE=1 selects D; WRITE=0 selects that bit's own Q feedback.</dd>
<dt>D → B → C → F: CLOCK</dt><dd>The rotated CLOCK input (D) retains the equal 16T/16T header. Actuator B drives the existing stroke amplifier C: about ±3.75 mm becomes ±9.375 mm. Its shortened output post is joined to rod F. The direction-changing controller linkage has gone; the stroke amplifier remains.</dd>
<dt>H–L: one bit</dt><dd>Master I and slave J now sit at the same height. Straight interstage, POWER and Q-feedback routes replace the stepped routes. CLOCK rod F drives bar K through a roller-ended bellcrank. On the rising edge, K withdraws master drive and permits the master to lock, then unlocks and drives the slave from the master. Falling clock reverses the sequence. Q exits at L.</dd>
<dt>M–O: shared power, independent storage</dt><dd>Local input M replaces the long incoming POWER route. Shafts N and O supply opposite rotation directions to both storage clutches; the slave’s duplicate reversing pair is removed. The M and Q storage outputs remain independent. The three-gear POWER train has moved 38 mm along its shafts, away from the former bearing obstruction. Transmission bearing walls are separate parts with two spaced mounting pins each. Their exported print orientations put axle holes normal to the bed. Front and detached rear bearing cheeks share two LEGO 6558 3L joining pins. Both controller return bands are restored, and elastic loops follow their moving anchors.</dd><dt>Friction, mass and strength</dt><dd>Four plain-bearing roller followers per bit replace bare axle contact in the bellcrank slots. The bush is keyed to its axle; the axle can turn in the crank's circular hole. Crank arms have 6 mm webs and enlarged pivot bosses. Rod guides now have removable paired-pin caps. Broader mounting pads and thicker fixture webs improve nominal sections; strength and actuation force still require physical testing.</dd>
<dt>Force still required</dt><dd>For N bits, ideal WRITE actuator force remains N times a selector's required force. The shorter WRITE rod stroke raises its rod load relative to the former 1.5× travel arrangement. CLOCK's 2.5× stroke amplification still requires roughly 2.5N times a local bar's force, before friction and inertia. Rolling, rod buckling, printed stiffness, preload and loaded operation need physical tests.</dd></dl>
<p>Labels identify the shared controller, power routes and the lowest bit. The animation uses inherited operation poses and does not simulate eight independent inputs or prove sufficient actuation force. Roller spin is not simulated.</p></section>"""
s=s.replace('<script id="data"',explanation+'<script id="data"')
label_js=(R/'Source/wall_labels.js').read_text()
s=s.replace('function draw(){',label_js+'\nfunction draw(){')
s=s.replace("document.querySelector('#readout').textContent=`${rows}","drawAnnotations(f,sx,sy,ratio);document.querySelector('#readout').textContent=`${rows}")
s=s.replace('window.onresize=draw;',"document.querySelector('#labelMode').onchange=draw;window.onresize=draw;")
# Animate elastic paths with their two moving anchors, not a rigid reference loop.
start=s.index('for(const p of parts){let a=v.subarray(')
end=s.index('let az=',start)
original=s[start:end]
normal_start=original.index('out=new Float32Array')
normal_end=original.index('p.buffer=gl.createBuffer()')
normal=original[normal_start:normal_end]
normal='let '+normal.replace('out=new Float32Array','out=new Float32Array',1)
s=s[:start]+'''function interleaved(a){'''+normal+'''return out;}
for(const p of parts){p.buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,p.buffer);gl.bufferData(gl.ARRAY_BUFFER,interleaved(v.subarray(p.offset,p.offset+p.vertices*3)),gl.DYNAMIC_DRAW);}
for(const e of D.elastic){const raw=Uint8Array.from(atob(e.geometry),c=>c.charCodeAt(0));const stream=new Blob([raw]).stream().pipeThrough(new DecompressionStream('gzip'));parts[e.part].elasticFrames=new Float32Array(await new Response(stream).arrayBuffer());}
'''+s[end:]
s=s.replace('const f=frames[index];for(let row=', 'const f=frames[index];for(const p of parts){if(p.elasticFrames){const n=p.vertices*3;gl.bindBuffer(gl.ARRAY_BUFFER,p.buffer);gl.bufferSubData(gl.ARRAY_BUFFER,0,interleaved(p.elasticFrames.subarray(index*n,(index+1)*n)));}}for(let row=')
s=s.replace('Elastic loops are shown in reference shape.', 'Elastic loops follow their animated anchors; elastic forces are not simulated.')
s=s.replace('Equal-tooth gear routes preserve engaged speed magnitude.', 'Shared opposite-running POWER shafts supply both storage clutches. Equal-tooth gear routes preserve engaged speed magnitude.')
# Wall-only navigation; keep other assembly viewers unchanged.
s=s.replace('<button id="front">', '<label>Drag <select id="navMode"><option value="rotate">Rotate</option><option value="pan">Move</option></select></label><button id="resetView">Reset view</button><button id="front">')
s=s.replace('Drag to orbit and scroll to zoom.', 'Drag to rotate; Shift-drag or right-drag to move. Select Move for ordinary dragging. Scroll to zoom; on touchscreens use two fingers to move and pinch to zoom. Reset view recentres the model.')
s=s.replace('uniform vec2 scale;', 'uniform vec2 scale;uniform vec2 pan;')
s=s.replace('w.x*scale.x,w.z*scale.y,', 'w.x*scale.x+pan.x,w.z*scale.y+pan.y,')
s=s.replace("'spin','ang','scale','color'", "'spin','ang','scale','pan','color'")
s=s.replace('gl.uniform2f(loc.scale,sx,sy);', 'gl.uniform2f(loc.scale,sx,sy);gl.uniform2f(loc.pan,panX,panY);')
s=s.replace('(1+u*sx)*', '(1+u*sx+panX)*').replace('(1-zz*sy)*', '(1-zz*sy-panY)*')
start=s.index('let drag;canvas.onpointerdown=')
end=s.index('document.querySelector(\'#rows\').onchange',start)
s=s[:start]+(R/'Source/wall_navigation.js').read_text()+'\n'+s[end:]
# Inspect the printable frames without the working mechanism obscuring them.
s=s.replace('<label>Labels ', '<label>Show <select id="partView"><option value="assembly">Complete mechanism</option><option value="frames">Frames only</option></select></label><label>Labels ')
s=s.replace("const p=parts[i];if(row>0", "const p=parts[i];if(document.querySelector('#partView').value==='frames'&&!p.id.includes('coordinated chassis'))continue;if(row>0")
s=s.replace('drawAnnotations(f,sx,sy,ratio);', "if(document.querySelector('#partView').value!=='frames')drawAnnotations(f,sx,sy,ratio);")
s=s.replace('window.onresize=draw;', "document.querySelector('#partView').onchange=draw;window.onresize=draw;")
s=s.replace('Design and assembly notes</a>', 'Design and assembly notes</a> · <a href="Wall%20register/Assembly%20and%20print%20revision.md">New assembly sequence</a> · <a href="Wall%20register/Flat%20frame%20printing.md">Flat frame printing</a> · <a href="Wall%20register/Axle-hole%20printing.md">Axle-hole printing</a> · <a href="Wall%20register/Friction-pin%20connections.md">Friction-pin connections</a>')
(R/'Wall register.html').write_text(s)
print('Published',len(p),'parts,',len(ff),'frames')
