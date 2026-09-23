from pathlib import Path
import runpy,json,re,gzip,base64,numpy as np
from direct_cam_math import cam_lift,bolt_lift,CREST,TANGENT,INTERCEPT
R=Path(__file__).resolve().parents[1];O=R/'Planar register'
n=runpy.run_path(str(R/'Source/publish_working_layout.py'));D=n['D'];js=n['js'];head=n['head']
def band_vertices(up):
 outer=[];inner=[]
 for z,start in [(49.3,np.pi),(57.3+up,0)]:
  for j in range(33):
   t=start+j*np.pi/32;outer.append([18.6+3.1*np.cos(t),z+3.1*np.sin(t)]);inner.append([18.6+2.5*np.cos(t),z+2.5*np.sin(t)])
 vs=[]
 def pt(p,x):return [x,p[0],p[1]]
 def quad(a,b,c,d):vs.extend([a,b,c,a,c,d])
 for i in range(len(outer)):
  j=(i+1)%len(outer)
  for x in [30.1,31.1]:quad(pt(outer[i],x),pt(outer[j],x),pt(inner[j],x),pt(inner[i],x))
  for ar in [outer,inner]:quad(pt(ar[i],30.1),pt(ar[j],30.1),pt(ar[j],31.1),pt(ar[i],31.1))
 return np.array(vs)
old=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3);arrays=[]
for p in D['parts']:
 a=band_vertices(0) if p['motion']=='lock-band' else old[p['offset']//3:p['offset']//3+p['vertices']].copy();p['offset']=sum(x.size for x in arrays);p['vertices']=len(a);arrays.append(a)
v=np.concatenate(arrays);D['geometry']=base64.b64encode(gzip.compress(v.astype('<f4').tobytes())).decode();D['bounds']=[(v.min(0)-[5,0,5]).tolist(),(v.max(0)+[5,0,10]).tolist()]
frames=json.loads((R.parent/'work/register-mux-reference/multiplexer/Switching trace.json').read_text())['frames']
def beta(q):return min(frames,key=lambda f:abs(f['q']-q))['b']
D['trace']=[]
for m0,m1,e0,e1,label in [(3.75,3.75,-3.75,3.75,'WRITE: ramp withdraws bolt before D connects'),(3.75,-3.75,3.75,3.75,'WRITE D = 1'),(-3.75,-3.75,3.75,-3.75,'HOLD: D disconnects before bolt inserts'),(-3.75,-3.75,-3.75,-3.75,'HOLD Q = 1'),(-3.75,-3.75,-3.75,3.75,'WRITE: unlock'),(-3.75,3.75,3.75,3.75,'WRITE D = 0'),(3.75,3.75,3.75,-3.75,'HOLD Q = 0')]:
 for t in np.linspace(0,1,100):
  qm=float(m0+(m1-m0)*t);qe=float(e0+(e1-e0)*t);D['trace'].append(dict(q=qm,qm=qm,qe=qe,s=bolt_lift(qm,qe),angle=0,bm=beta(qm),be=beta(qe),b=0,w=0,g=0,segment=label))
camjs=f"function camLift(q){{let u=38-q,z;if(u<={CREST})z=65.1;else if(u<={TANGENT})z=61.5+Math.sqrt(Math.max(0,12.96-(u-{CREST})**2));else z={INTERCEPT}-1.8*u+3.6*Math.sqrt(4.24);return Math.max(0,z-57.3)}}"
js=js.replace('<script>','<script>'+camjs,1)
oldexpr='let a=Math.atan2(6.8,10)+Math.asin((qe-6.8)/Math.sqrt(146.24)),up=Math.max(0,54+18*Math.sin(a)+8*Math.cos(a)+3.6-57);'
assert oldexpr in js;js=js.replace(oldexpr,'let a=0,up=camLift(qe);')
a=js.index('function lockBand(up)');b=js.index('let az=-1.1',a);chunk=js[a:b].replace('41.9','49.3').replace('59.4','57.3').replace('34+','18.6+').replace('4.1*','3.1*').replace('3.3*','2.5*').replace('24.0','30.1').replace('25.2','31.1');js=js[:a]+chunk+js[b:]
head=re.sub(r'<details>.*?</details>','<details><summary>Design notes and validation limits</summary><p>Direct roller cam: orange plate lifts the gold bolt; the blue bearing cheek contains its guide. One rear backbone carries the axle supports. The crank, pivot brackets and long roller axle are removed. Original carriage bearing print faces are retained. D and WRITE enter at the right; constant 1 power and Q are at the left. Q has unequal speed in its two directions. Prescribed motion; physical 0.1 Nm qualification remains outstanding.</p></details>',head,flags=re.S)
js=js.replace("$('#ports').innerHTML=portMarkup;","$('#ports').innerHTML=$('#part').value?'':portMarkup;")
head=head.replace('width="190" height="190"','width="84" height="84"')
# Names for the added moving mechanism, anchored to its current pose.
head=head.replace('aria-label="Input and output axle labels"','aria-label="Mechanism labels"')
head=head.replace('<label>Inspect ', '<label>Labels <select id="labelmode"><option value="moving">New moving parts</option><option value="ports">Inputs / outputs</option><option value="none">None</option></select></label><label>Inspect ')
head=head.replace('<div id="error">', '<div id="moving-key"><b>New moving parts</b><p><b>A · Cam plate</b> — orange ramp, carried by the WRITE carriage; lifts the lock.<br><b>B · Cam roller</b> — LEGO half-bush that follows the ramp; its axle and retaining bush move with the bolt.<br><b>C · Lock bolt</b> — gold vertical slider; enters a locking pocket during HOLD.<br><b>D · Keeper</b> — two locking pockets added to the teal memory carriage; moves with the stored bit.<br><b>E · Return band</b> — purple elastic band that pulls the bolt into the keeper.</p><p>The bolt guide and grey-green frame are fixed. The two cam attachment friction pins travel with A.</p></div><div id="error">')
labels=r"""let movingMarkup='';
if($('#labelmode').value==='moving'&&!$('#part').value){
for(let [name,point,lx,ly,color] of [
['A · Cam plate',[63+f.qe,31,51],.76,.31,'#985423'],
['B · Cam roller',[38,27.6,57.3+f.s],.58,.08,'#3e4b52'],
['C · Lock bolt',[42,18,60+f.s],.26,.12,'#8c6a0c'],
['D · Keeper',[45+f.qm,24.8,42.1],.76,.67,'#126c77'],
['E · Return band',[30.6,21.7,53.3+f.s/2],.20,.29,'#903e79']]){
let v=point.map((n,i)=>n-[48,17,24][i]),dot=a=>a.reduce((sum,n,i)=>sum+n*v[i],0),x=(1+fit*dot(right)+px)*w/2,y=(1-fit*w/h*dot(up)-py)*h/2,tx=lx*w,ty=ly*h;
movingMarkup+=`<line x1="${tx}" y1="${ty}" x2="${x}" y2="${y}" stroke="${color}" stroke-width="1.5"/><circle cx="${x}" cy="${y}" r="4" fill="${color}" stroke="white"/><rect x="${tx-69}" y="${ty-13}" width="138" height="26" rx="4" fill="#fffef7" stroke="${color}"/><text x="${tx}" y="${ty+4}" text-anchor="middle" fill="${color}" font-size="12" font-weight="700">${name}</text>`;
}}
$('#ports').innerHTML=$('#part').value?'':$('#labelmode').value==='moving'?movingMarkup:$('#labelmode').value==='ports'?portMarkup:'';
$('#moving-key').hidden=$('#labelmode').value!=='moving';
"""
js=js.replace("$('#ports').innerHTML=$('#part').value?'':portMarkup;",labels)
js=js.replace('new ResizeObserver(schedule).observe(cv)',"$('#labelmode').onchange=schedule;new ResizeObserver(schedule).observe(cv)")
# Replace prescribed sequential switching with simultaneous contact-event audit cases.
audit=json.loads((O/'Transition timing audit.json').read_text())
D['auditCases']=audit['cases']
for case in D['auditCases']:
 for f in case['frames']:f.update(bm=beta(f['qm']),be=beta(f['qe']))
D['trace']=next(c['frames'] for c in D['auditCases'] if c['initial']==[1,1,1] and c['target']==[0,0] and c['profile']=='equal carriage speed / mesh contact')
head=head.replace('<option value="moving">New moving parts</option>', '<option value="both">Parts + inputs / outputs</option><option value="moving">New moving parts</option>')
js=js.replace("$('#labelmode').value==='moving'&&!$('#part').value", "['both','moving'].includes($('#labelmode').value)&&!$('#part').value")
js=js.replace("$('#labelmode').value==='moving'?movingMarkup:", "$('#labelmode').value==='both'?portMarkup+movingMarkup:$('#labelmode').value==='moving'?movingMarkup:")
js=js.replace("$('#labelmode').value!=='moving'", "!['both','moving'].includes($('#labelmode').value)")
# Put port labels around the outer margin, away from mechanism callouts.
js=js.replace(".12,.40,'#126c77'", ".12,.57,'#126c77'").replace(".14,.79,'#126c77'", ".14,.91,'#126c77'").replace(".83,.79,'#985423'", ".83,.91,'#985423'").replace(".67,.16,'#985423'", ".82,.19,'#985423'")
head=re.sub(r'<div class="tools"><button id="d0">.*?</div>', '<div class="tools"><label>Transition <select id="audit-transition"></select></label><label>Timing example <select id="audit-profile"></select></label></div><p style="margin:6px 0;color:#873f25"><b>Not ready to print:</b> simultaneous D reversal and WRITE→HOLD can leave the memory between pockets. <a href="Transition timing audit.md">Read the timing audit</a>. Examples use normalized time and assumed rates; they are not measured motor timing.</p>',head)
head=head.replace('Run write / hold cycle','Run selected transition')
a=js.index('let desiredData=');b=js.index("$('#labelmode').onchange",a)
controls=r"""let transitions=[...new Set(D.auditCases.map(c=>JSON.stringify([...c.initial,...c.target])))];
$('#audit-transition').innerHTML=transitions.map(k=>{let [d,w,q,dd,ww]=JSON.parse(k);return `<option value='${k}'>D ${d}→${dd}, WRITE ${w}→${ww}; initial Q=${q}</option>`}).join('');
let profiles=[...new Set(D.auditCases.map(c=>c.profile))];$('#audit-profile').innerHTML=profiles.map((p,i)=>`<option value="${i}">${p}</option>`).join('');
function auditSelect(){let k=JSON.parse($('#audit-transition').value),p=profiles[+$('#audit-profile').value];D.trace=D.auditCases.find(c=>JSON.stringify([...c.initial,...c.target])===JSON.stringify(k)&&c.profile===p).frames;idx=0;playing=false;last=0;$('#run').textContent='Run selected transition';$('#timeline').max=D.trace.length-1;schedule()}
$('#audit-transition').value=JSON.stringify([1,1,1,0,0]);$('#audit-transition').onchange=$('#audit-profile').onchange=auditSelect;auditSelect();
"""
js=js[:a]+controls+js[b:]
js=js.replace('Run write / hold cycle','Run selected transition')
js=js.replace("if(mo==='clutch-ring')t=[bankSign*Math.sign(q)*Math.max(Math.abs(q)-.4,0),0,0];", "if(mo==='clutch-ring')t=[p.bank==='Memory'?f.rm:f.re,0,0];")
head=head.replace('displayed shaft rotation is omitted.', 'shaft rotation and tooth forces are not simulated. Contact indicates possible torque transfer, including partial engagement; angular take-up and loaded release need measurement.')
head=head.replace('Read the timing audit</a>.', 'Read the timing audit</a>. <a href="Cam%20mechanism%20audit.md">Cam/bolt load review: guide revision required</a>.')
(O/'Viewer.html').write_text(head+'<script type="application/json" id="data">'+json.dumps(D,separators=(',',':'))+'</script>'+js)
print('Published direct cam viewer')
