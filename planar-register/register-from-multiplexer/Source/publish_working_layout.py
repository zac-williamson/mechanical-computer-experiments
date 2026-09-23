from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
# Reuse the original self-contained viewer, with a physically constrained bolt trace.
s=(R/'Source/publish_reoriented.py').read_text()
s=s.replace("cmd=2*(qe-4.3)","angle=float(np.arctan2(6.8,10)+np.arcsin((qe-6.8)/np.sqrt(146.24)));cmd=max(0,54+18*np.sin(angle)+8*np.cos(angle)+3.6-57)")
s=s.replace('s=min(0,cmd),angle=float(np.arcsin(qe/10))','s=cmd,angle=angle')
s=s.replace('-4.3','-3.75').replace('4.3','3.75')
s=s.replace("if(mo==='bell-crank')[R,t]=rot('Y',f.angle,[20,0,-52.8]);","if(mo==='release-crank')[R,t]=rot('Y',f.angle,[60,0,54]);")
s=s.replace('Orange write actuator rotated 180°; blue carriage restored to its original length.','Gold: adjacent lock, release crank and elastic band. WRITE = 1 engages D; WRITE = 0 holds Q.')
s=s.replace('Reoriented starting layout. The rejected extended carriages and remote lock have been removed. A compact lock is not fitted yet.','Connected prototype: direct WRITE worm input, one-mesh inverted data, and swapped output power paths. The blue carriage keeps its original height. Motion is kinematic; physical friction and 0.1 Nm endurance are not proven.')
s=s.replace('Show carriage travel','Run write / hold cycle')
s=s.replace('gl.uniform3f(L.center,48,17,-14)','gl.uniform3f(L.center,48,17,24)').replace("D.bounds[k&4?1:0][2]+14","D.bounds[k&4?1:0][2]-24")
# Keep source-trace operating endpoints; do not force the memory into pockets at arbitrary ±4.3.
# Add the actual elastic band's changing capsule envelope to the viewer.
needle="frames=json.loads"
s=s.replace(needle,"""def band_geometry(up):
 outer=[];inner=[]
 for end in [0,1]:
  for j in range(33):
   t=(np.pi if end==0 else 0)+j*np.pi/32;c=[34,41.9] if end==0 else [34,59.4+up]
   outer.append([c[0]+4.1*np.cos(t),c[1]+4.1*np.sin(t)]);inner.append([c[0]+3.3*np.cos(t),c[1]+3.3*np.sin(t)])
 # Caps must face away from the other centre: lower goes left->bottom->right.
 outer=[];inner=[]
 for c,start in [([34,41.9],np.pi),([34,59.4+up],0)]:
  for j in range(33):
   t=start+j*np.pi/32;outer.append([c[0]+4.1*np.cos(t),c[1]+4.1*np.sin(t)]);inner.append([c[0]+3.3*np.cos(t),c[1]+3.3*np.sin(t)])
 vs=[]
 def pt(p,x):return [x,p[0],p[1]]
 def quad(a,b,c,d):vs.extend([a,b,c,a,c,d])
 for i in range(len(outer)):
  j=(i+1)%len(outer)
  for x in [24.0,25.2]:quad(pt(outer[i],x),pt(outer[j],x),pt(inner[j],x),pt(inner[i],x))
  for ar in [outer,inner]:quad(pt(ar[i],24.0),pt(ar[j],24.0),pt(ar[j],25.2),pt(ar[i],25.2))
 return np.array(vs)
a=band_geometry(0);parts.append(dict(id='Lock elastic band',bank='Lock',motion='lock-band',kind='elastic',color=[172,83,145],offset=sum(x.size for x in arrays),vertices=len(a)));arrays.append(a)
"""+needle)
# Insert a JS equivalent; coordinates follow the retained rectangular band lugs.
pos="start=js.index(\"if(mo==='carriage')\")"
s=s.replace(pos,"""js=js.replace('let az=-1.1', '''function lockBand(up){let o=[],inn=[];for(let [z,start]of[[41.9,Math.PI],[59.4+up,0]])for(let j=0;j<=32;j++){let t=start+j*Math.PI/32;o.push([34+4.1*Math.cos(t),z+4.1*Math.sin(t)]);inn.push([34+3.3*Math.cos(t),z+3.3*Math.sin(t)])}let v=[],pt=(p,x)=>[x,p[0],p[1]],quad=(a,b,c,d)=>v.push(...a,...b,...c,...a,...c,...d);for(let i=0;i<o.length;i++){let j=(i+1)%o.length;for(let x of[24.0,25.2])quad(pt(o[i],x),pt(o[j],x),pt(inn[j],x),pt(inn[i],x));for(let a of[o,inn])quad(pt(a[i],24.0),pt(a[j],24.0),pt(a[j],25.2),pt(a[i],25.2))}return new Float32Array(v)}let az=-1.1''')
"""+pos)
s=s.replace("if(mo==='bolt')t=[0,0,f.s];", "if(mo==='bolt')t=[0,0,f.s];\nif(mo==='lock-band'){let v=lockBand(f.s);for(let[b,a]of[[p.vb,v],[p.nb,normals(v)]]){gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,a,gl.DYNAMIC_DRAW)}p.vertices=v.length/3;}")
s=s.replace("+' mm · Write '+f.qe.toFixed(1)+' mm'","+' mm · Write carriage '+f.qe.toFixed(1)+' mm · Bolt lift '+f.s.toFixed(1)+' mm'")
s=s.replace('D=dict(parts=parts,', 'D=dict(leverLookup=[beta(float(q)) for q in np.linspace(-4.6,4.6,461)],parts=parts,')
# Add user-operable D / WRITE controls to the same kinematic model.
s=s.replace('<div hidden>', '<div class="tools"><button id="d0">D = 0</button><button id="d1">D = 1</button><button id="write1">WRITE = 1</button><button id="write0">WRITE = 0 / HOLD</button></div><div hidden>')
client_controls="let desiredData=0,desiredWrite=0;\nfunction control(){let f=D.trace[Math.floor(idx)],qm=f.qm,qe=f.qe,list=[];for(let i=0;i<600;i++){let target=desiredWrite?3.75:-3.75;qe+=Math.sign(target-qe)*Math.min(Math.abs(target-qe),.04);let a=Math.atan2(6.8,10)+Math.asin((qe-6.8)/Math.sqrt(146.24)),up=Math.max(0,54+18*Math.sin(a)+8*Math.cos(a)+3.6-57);if(qe>.4&&up>=5.8){let mt=desiredData?-3.75:3.75;qm+=Math.sign(mt-qm)*Math.min(Math.abs(mt-qm),.04)}let blocked=Math.abs(qm)<3.3&&up<5.4;if(blocked)up=5.4;let Q=Math.abs(qm)>=3.3?(qm<0?'1':'0'):'transition';list.push({q:qm,qm,qe,s:up,angle:a,bm:D.leverLookup[Math.max(0,Math.min(460,Math.round((qm+4.6)*50)))],be:D.leverLookup[Math.max(0,Math.min(460,Math.round((qe+4.6)*50)))],b:0,w:0,g:0,segment:'D = '+desiredData+' · WRITE = '+desiredWrite+' · Q = '+Q+(blocked?' · interrupted write: bolt rests on keeper':'')})}D.trace=list;idx=0;playing=true;last=0;$('#timeline').max=list.length-1;$('#run').textContent='Pause';schedule()}\n$('#d0').onclick=()=>{desiredData=0;control()};$('#d1').onclick=()=>{desiredData=1;control()};$('#write1').onclick=()=>{desiredWrite=1;control()};$('#write0').onclick=()=>{desiredWrite=0;control()};new ResizeObserver(schedule).observe(cv)"
s=s.replace("(O/'Viewer.html').write_text", "js=js.replace('new ResizeObserver(schedule).observe(cv)', "+repr(client_controls)+")\n(O/'Viewer.html').write_text")

# Persistent axle-end callouts follow the camera and identify external ports.
labels_js="""let portMarkup='';for(let [name,point,lx,ly,color] of [['Q · OUTPUT',[-44,10.2,0],.12,.40,'#126c77'],['1 · POWER IN',[-44,10.2,-16],.14,.79,'#126c77'],['D · DATA IN',[140.8,10.2,0],.83,.79,'#985423'],['WRITE · IN',[140.8,10.2,32],.67,.16,'#985423']]){let v=point.map((n,i)=>n-[48,17,24][i]),dot=a=>a.reduce((sum,n,i)=>sum+n*v[i],0),x=(1+fit*dot(right)+px)*w/2,y=(1-fit*w/h*dot(up)-py)*h/2,tx=lx*w,ty=ly*h;portMarkup+=`<line x1="${tx}" y1="${ty}" x2="${x}" y2="${y}" stroke="${color}" stroke-width="1.5"/><circle cx="${x}" cy="${y}" r="4" fill="${color}"/><rect x="${tx-62}" y="${ty-12}" width="124" height="24" rx="4" fill="#fffef7" stroke="${color}"/><text x="${tx}" y="${ty+4}" text-anchor="middle" fill="${color}" font-size="12" font-weight="700">${name}</text>`}$('#ports').setAttribute('viewBox',`0 0 ${w} ${h}`);$('#ports').innerHTML=portMarkup;"""
s=s.replace('<canvas id="view"></canvas>', '<canvas id="view"></canvas><svg id="ports" aria-label="Input and output axle labels" style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none"></svg>')
s=s.replace("(O/'Viewer.html').write_text", 'js=js.replace('+repr("$('#axes').innerHTML=axisSvg;")+', '+repr("$('#axes').innerHTML=axisSvg;"+labels_js)+')\n'+"(O/'Viewer.html').write_text")
s=s.replace('Motion is kinematic; physical friction and 0.1 Nm endurance are not proven.', 'Corrected memory drive: 8T → 8T → 16T for 1, 16T → 16T for 0. Q speed is half power speed for 1 and full power speed for 0. Motion is kinematic; 0.1 Nm endurance is unproven.')
s=s.replace('Drag to rotate · Shift-drag to pan · Scroll to zoom.', 'Inputs: D (data), WRITE (1 = write, 0 = hold), and constant anticlockwise 1 power. Output: Q. Directions viewed from +X toward the origin. Drag to rotate · Shift-drag to pan · Scroll to zoom.')

s=s.replace(';angle=float', ';qm=-qm;qe=-qe;angle=float')
s=s.replace('The blue carriage keeps its original height.', 'The bearing carriages retain their original bed faces. The orange linkage is a separate flat part on two LEGO friction pins; its rear extent is reduced by 7.4 mm.')
s=s.replace('<p>Teal:', '<details><summary>Design notes and validation limits</summary><p>Teal:').replace('</p><div class="tools">', '</p></details><div class="tools">').replace('padding:22px','padding:12px').replace('height:65vh','height:68vh').replace('margin:12px 0','margin:6px 0')
exec(compile(s,str(R/'Source/publish_reoriented.py'),'exec'))
