from pathlib import Path
import runpy,json,re,shutil
S=Path(__file__).resolve().parent;O=S.parents[1]/'work/integrated-cam-development';T=S.parent/'Planar register'
runpy.run_path(str(S/'publish_integrated_cam.py'))
s=(O/'Viewer.html').read_text();mat=re.search(r'<script type="application/json" id="data">(.*?)</script>',s,re.S);D=json.loads(mat.group(1));model=json.loads((O/'Coupled operation.json').read_text());D['auditCases']=model['cases'];D['trace']=model['cases'][0]['frames'];D['kinematics']={k:v for k,v in model.items() if k!='cases'}
for p in D['parts']:
 n=p['id'];bank=p.get('bank');source=n.split(' — ')[-1];field=None;axis='X';ratio=1;center=[0,10.2,0 if bank=='Memory' else 16]
 if bank in ['Memory','Write'] and p.get('kind')=='native':
  if source in ['U022','reaction-stop-axle','reaction-retainer']:field='gm' if bank=='Memory' else 'ge';axis='Y';center=[0 if bank=='Memory' else -85.0,10.2,24 if bank=='Memory' else 40]
  elif source in ['pivot-stop-axle','pivot-retainer']:field='bm' if bank=='Memory' else 'be';axis='Y';center=[13.192323604+(0 if bank=='Memory' else -85.0),10.2,32.128448698+(0 if bank=='Memory' else 16)]
  elif source in ['C-shaft','U015','U072','selector-right-retainer']:field='wm' if bank=='Memory' else 'we';center=[0,10.2,16 if bank=='Memory' else 32]
  elif 'pin' in n:pass
  elif bank=='Memory':
   if 'idler' in n:field='power';ratio=-1;center=[0,10.2+33.75**.5,-10.5]
   elif 'input' in n or 'power axle' in n:field='power';center=[0,10.2,-16]
   elif source=='L072':field='power';ratio=-1
   elif source=='L102':field='power';ratio=.5
   elif source in ['L097','L099','L069','L105','O-left','O-right','O-shaft','O-left 5L axle','O-right 5L axle']:field='output'
  else:
   if 'B-input' in n or source=='B-shaft':field='data';center=[0,10.2,0]
   elif source=='L102':field='data';ratio=-1
   elif source in ['L097','L099','L069','L105','O-left','O-right','O-shaft','O-left 5L axle','O-right 5L axle']:field='wm'
 if n.startswith('2L LEGO axle joiner'):field='wm';center=[0,10.2,16]
 if n.startswith('Cam roller'):field='roller';axis='Y';center=[-5.05,0,53.3]
 if field:p['joint']=dict(field=field,axis=axis,center=center,ratio=ratio)
D['sharedPoseVersion']='coupled-v1'
s=s[:mat.start(1)]+json.dumps(D,separators=(',',':'))+s[mat.end(1):]
a=s.index("let q=p.bank==='Memory'?f.qm:f.qe,bankSign=1;");b=s.index('gl.uniformMatrix3fv(L.model',a)
s=s[:a]+'''let q=p.bank==='Memory'?f.qm:f.qe;
if(p.joint){let j=p.joint;[R,t]=rot(j.axis,f[j.field]*j.ratio*Math.PI/180,j.center)}
if(mo==='carriage'||mo==='worm')t[0]+=q;
if(mo==='clutch-ring')t[0]+=p.bank==='Memory'?f.rm:f.re;
if(mo==='bolt')t[2]+=f.s;
if(mo==='rocker'){let mem=p.bank==='Memory';[R,t]=rot('Y',(mem?f.bm:f.be)*Math.PI/180,[13.192323604+(mem?0:-85.0),10.2,32.128448698+(mem?0:16)])}
if(mo==='lock-band'){let v=lockBand(f.s);for(let[b,a]of[[p.vb,v],[p.nb,normals(v)]]){gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,a,gl.DYNAMIC_DRAW)}p.vertices=v.length/3;}
''' + s[b:]
s=s.replace('This animation prescribes carriage travel; it does not simulate the gear drive or prove operation.','Worm input rotation now drives carriage travel through gear/lever contact constraints. The same recorded shaft angles drive this viewer and the pose checks. This is a rigid kinematic model, not physical qualification.')
s=s.replace('shaft rotation and tooth forces are not simulated.','shaft rotation is coupled to carriage travel. Tooth forces and deformation are not solved.')
s=s.replace('The source multiplexer lever pose is approximated from its contact trace;','The lever pose is constrained by the native tooth and carriage profiles;')
s=s.replace('Examples use normalized time and assumed rates; they are not measured motor timing.','Equal 60 rpm inputs illustrate operation. Clutch take-up values are sensitivity assumptions, not measured backlash limits.')
s=s.replace("f.segment+' · Memory '","f.segment+' · '+f.memory_mode+' · Memory '")
s=s.replace('<div id="status"></div>','<div id="status"></div><details><summary>Motion readout</summary><div id="motion-readout"></div></details>')
s=s.replace("if(playing)schedule()}\nfunction schedule", "$('#motion-readout').textContent='WRITE input '+f.we.toFixed(1)+'°; WRITE reaction gear '+f.ge.toFixed(1)+'° — '+f.write_mode+'. Memory worm '+f.wm.toFixed(1)+'°; memory reaction gear '+f.gm.toFixed(1)+'° — '+f.memory_mode+'. Clutch angular take-up remaining '+f.takeup_remaining.toFixed(1)+'°.';if(playing)schedule()}\nfunction schedule")
s=s.replace('<a href="Transition timing audit.md">Read the timing audit</a>','<a href="Collision audit.html">Read the current motion and collision audit</a>')
s=s.replace('Planar register · integrated cam','Planar register · detachable bolt guide')
# Expose the exact displayed joint transforms as data for independent Python checking.
(O/'Coupled viewer parts.json').write_text(json.dumps(D['parts'],indent=2));(O/'Viewer.html').write_text(s);(T/'Viewer.html').write_text(s)
for part in json.loads((O/'printed-parts.json').read_text()):shutil.copy2(O/(part['id']+'.stl'),T/(part['id']+'.stl'))
for name in ['printed-parts.json','Integrated cam parameters.json','hardware.json','hardware.npz','Coupled operation.json','Coupled viewer parts.json','Keyed hardware alignment.json','Connected gear phases.json','Clutch axle stop correction.json']:
 shutil.copy2(O/name,T/name)
print('Published coupled viewer:',len(D['parts']),'parts;',len(model['cases']),'cases')
