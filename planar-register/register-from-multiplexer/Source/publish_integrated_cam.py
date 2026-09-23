from pathlib import Path
import json,re,gzip,base64,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R.parent/'work/integrated-cam-development';B=R.parent/'work/register-before-left-cam'
s=(B/'Viewer.html').read_text();match=re.search(r'<script type="application/json" id="data">(.*?)</script>',s,re.S);D=json.loads(match.group(1));pars=json.loads((O/'Integrated cam parameters.json').read_text());parts=[];arrays=[]
for p in json.loads((O/'printed-parts.json').read_text()):
 a=trimesh.load(O/(p['id']+'.stl')).triangles.reshape(-1,3);color=[134,149,134] if p['motion']=='fixed' else [29,147,157] if p['bank']=='Memory' else [190,117,71] if p['bank']=='Write' else [211,160,46];parts.append(dict(**p,kind='structure' if p['motion']=='fixed' else 'printed',color=color,offset=sum(a.size for a in arrays),vertices=len(a)));arrays.append(a)
v=np.load(O/'hardware.npz')['vertices']
for p in json.loads((O/'hardware.json').read_text()):
 a=v[p['offset']//3:p['offset']//3+p['vertices']];parts.append(dict(p,color=p.get('color',[66,79,91]),kind=p.get('kind','native'),offset=sum(a.size for a in arrays)));arrays.append(a)
def band(up):
 outer=[];inner=[]
 for z,start in [(50,np.pi),(69+up,0)]:
  for j in range(33):
   t=start+j*np.pi/32;outer.append([-5.05+3.1*np.cos(t),z+3.1*np.sin(t)]);inner.append([-5.05+2.5*np.cos(t),z+2.5*np.sin(t)])
 vs=[]
 def pt(p,y):return [p[0],y,p[1]]
 def quad(a,b,c,d):vs.extend([a,b,c,a,c,d])
 for i in range(len(outer)):
  j=(i+1)%len(outer)
  for y in [32.5,33.5]:quad(pt(outer[i],y),pt(outer[j],y),pt(inner[j],y),pt(inner[i],y))
  for ar in [outer,inner]:quad(pt(ar[i],32.5),pt(ar[j],32.5),pt(ar[j],33.5),pt(ar[i],33.5))
 return np.array(vs)
a=band(0);parts.append(dict(id='Lock elastic band',bank='Lock',motion='lock-band',kind='elastic',color=[172,83,145],offset=sum(a.size for a in arrays),vertices=len(a)));arrays.append(a)
v=np.concatenate(arrays);D.update(parts=parts,geometry=base64.b64encode(gzip.compress(v.astype('<f4').tobytes())).decode(),bounds=[(v.min(0)-[5,0,5]).tolist(),(v.max(0)+[5,0,8]).tolist()])
def lift(q):
 r=3.6;c=pars['crest'];u=-5.05-q;t=c+r*1.2/np.sqrt(2.44);z=pars['high']+r if u<=c else pars['high']+np.sqrt(max(0,r*r-(u-c)**2)) if u<=t else pars['intercept']-1.2*u+r*np.sqrt(2.44);return max(0,z-53.3)
D['auditCases']=json.loads((O/'Transition timing audit.json').read_text())['cases']
source_frames=json.loads((R.parent/'work/register-mux-reference/multiplexer/Switching trace.json').read_text())['frames']
lookup={}
def beta(q):
 key=round(q,5)
 if key not in lookup:lookup[key]=min(source_frames,key=lambda f:abs(f['q']-q))['b']
 return lookup[key]
for c in D['auditCases']:
 for f in c['frames']:f.update(bm=beta(f['qm']),be=beta(f['qe']))
D['trace']=D['auditCases'][0]['frames']
for case in D['auditCases']:
 for f in case['frames']:f['s']=max(lift(f['qe']),3.2 if abs(f['qm'])<3.0 else 0)
for f in D['trace']:f['s']=max(lift(f['qe']),3.2 if abs(f['qm'])<3.0 else 0)
s=s[:match.start(1)]+json.dumps(D,separators=(',',':'))+s[match.end(1):]
a=s.index('function camLift(');b=s.index('function ',a+10);s=s[:a]+f"function camLift(q){{let u=-5.05-q,z;if(u<={pars['crest']})z=58.7;else if(u<={pars['crest']+3.6*1.2/np.sqrt(2.44)})z=55.1+Math.sqrt(Math.max(0,12.96-(u-({pars['crest']}))**2));else z={pars['intercept']}-1.2*u+3.6*Math.sqrt(2.44);return Math.max(0,z-53.3)}}\n"+s[b:]
# Restore declarations between injected camLift and first ordinary function from original template.
# camLift was immediately followed by viewer setup, so replace only its balanced body instead above.
orig=(B/'Viewer.html').read_text();a0=orig.index('function camLift(');start=orig.index('{',a0);depth=1;i=start+1
while depth:
 depth+=(orig[i]=='{')-(orig[i]=='}');i+=1
oldtail=orig[i:orig.index('function ',a0+10)];pos=s.index('function ',s.index('function camLift(')+10);s=s[:pos]+oldtail+s[pos:]
a=s.index('function lockBand(');b=s.index('let az=-1.1',a)
s=s[:a]+'''function lockBand(up){let o=[],inn=[];for(let [z,start]of[[50,Math.PI],[69+up,0]])for(let j=0;j<=32;j++){let t=start+j*Math.PI/32;o.push([-5.05+3.1*Math.cos(t),z+3.1*Math.sin(t)]);inn.push([-5.05+2.5*Math.cos(t),z+2.5*Math.sin(t)])}let v=[],pt=(p,y)=>[p[0],y,p[1]],quad=(a,b,c,d)=>v.push(...a,...b,...c,...a,...c,...d);for(let i=0;i<o.length;i++){let j=(i+1)%o.length;for(let y of[32.5,33.5])quad(pt(o[i],y),pt(o[j],y),pt(inn[j],y),pt(inn[i],y));for(let a of[o,inn])quad(pt(a[i],32.5),pt(a[j],32.5),pt(a[j],33.5),pt(a[i],33.5))}return new Float32Array(v)}''' +s[b:]
s=s.replace('[109.992323604,10.2,48.128448698]','[-71.807676396,10.2,48.128448698]')
s=s.replace('gl.uniform3f(L.center,48,17,24)','gl.uniform3f(L.center,-40,17,27)').replace('D.bounds[k&1?1:0][0]-48','D.bounds[k&1?1:0][0]+40').replace('D.bounds[k&4?1:0][2]-24','D.bounds[k&4?1:0][2]-27').replace('[48,17,24]','[-40,17,27]')
s=s.replace('[140.8,10.2,0]','[-129,10.2,0]').replace('[140.8,10.2,32]','[-129,10.2,32]').replace("[-44,10.2,0]","[44.2,10.2,0]").replace("[-44,10.2,-16]","[44,10.2,-16]")
s=s.replace('[63+f.qe,31,51]','[-36+f.qe,31,52]').replace('[38,27.6,57.3+f.s]','[-5.05,27.6,53.3+f.s]').replace('[42,18,60+f.s]','[.95,18,64+f.s]').replace('[45+f.qm,24.8,42.1]','[-9+f.qm,22.4,44]').replace('[30.6,21.7,53.3+f.s/2]','[-5.05,33,58.5+f.s/2]')
s=s.replace('A · Cam plate','A · Integral cam').replace('D · Keeper','D · Locking pockets').replace('The two cam attachment friction pins travel with A.','The cam is part of the orange carriage; no cam attachment pins are needed.').replace('two locking pockets added to the teal memory carriage','two locking pockets cut into the teal memory carriage').replace('blue bearing cheek contains its guide','base frame carries its longer guide')
s=s.replace('Cam/bolt load review: guide revision required','Previous revision load review (superseded geometry)')
s=s.replace('<h1>Planar 1-bit register</h1>','<h1>Integrated cam revision — development</h1><p class="note">Integrated carriage cam, recessed locking pockets and base-supported guide. DO NOT PRINT THE ASSEMBLY: functional validation is incomplete. This animation prescribes carriage travel; it does not simulate the gear drive or prove operation.</p>')
s=re.sub(r'<details>.*?</details>','<details><summary>Design notes and validation limits</summary><p>WRITE has moved to the opposite X side. The cam is integral to the original bearing-flat carriage half. Two pockets are cut into the memory roof. A two-level bolt guide is supported by open ribs from the base. D and WRITE enter from the left; POWER and Q are accessible on the right. Conditional friction and bending screens are documented in README.md. The simultaneous closing-data race remains a separate latch-timing limitation.</p></details>',s,flags=re.S)
s=s.replace('Cam%20mechanism%20audit.md','README.md').replace('Previous revision load review (superseded geometry)','Revision checks and print-test instructions')
s=s.replace("$('#audit-transition').value=JSON.stringify([1,1,1,0,0])","$('#audit-transition').value=JSON.stringify([0,0,0,0,1])")
s=s.replace('Integrated cam revision — development','Planar register · integrated cam')
s=s.replace('height:68vh','height:58vh').replace('min-height:380px','min-height:320px')
s=s.replace(".12,.57,'#126c77'", ".88,.57,'#126c77'").replace(".14,.91,'#126c77'", ".86,.91,'#126c77'").replace(".83,.91,'#985423'", ".12,.91,'#985423'").replace(".82,.19,'#985423'", ".16,.19,'#985423'")
s=s.replace(".76,.31,'#985423'",".46,.28,'#985423'").replace(".58,.08,'#3e4b52'",".76,.08,'#3e4b52'").replace(".26,.12,'#8c6a0c'",".80,.31,'#8c6a0c'").replace(".20,.29,'#903e79'",".46,.10,'#903e79'")
s=s.replace('<div class="tools"><label>Transition ', '<details><summary>Timing examples · 12 input transitions</summary><div class="tools"><label>Transition ')
s=s.replace('they are not measured motor timing.</p>', 'they are not measured motor timing.</p></details>')
s=s.replace("['A · Integral cam',", "['F · Drive coupling',[-40.4,10.2,16],.24,.48,'#3e4b52'],\n['A · Integral cam',")
(O/'Viewer.html').write_text(s);print('Published development viewer',np.ptp(v,axis=0))
