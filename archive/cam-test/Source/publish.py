from pathlib import Path
import sys,json,re,gzip,base64,math
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parent;import os;OUT=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));D=json.loads((OUT/'Assembly manifest.json').read_text());sys.path.insert(0,str(ROOT));from render_ldraw import LDraw,render
lib=LDraw(str(Path(os.environ.get('LDRAW_DIR','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'));source=(ROOT.parents[1]/'multiplexer/Viewer.html').read_text();trace=json.loads((ROOT.parents[1]/'multiplexer/Actuator poses.json').read_text());ends={str(k):min(trace,key=lambda p:abs(p['q']-(4.325 if k==0 else -4.35))) for k in [0,1]};old=json.loads(re.search(r'id="data">(.*?)</script>',source,re.S)[1]);colors={p['id']:p['color'] for p in old['parts']};parts=[];chunks=[];offset=0
for p in D['prints']:
 t=trimesh.load(OUT/p['path']).triangles;col=[99,129,119] if p['motion']=='fixed' else [47,121,139]
 if 'lever' in p['id']:col=[184,113,61]
 if 'Write' in p['id']:col=[205,157,58]
 if 'HOLD bolt'==p['id']:col=[161,95,145]
 a=t.astype('<f4').reshape(-1);parts.append(dict(id=p['id'],actor=p['actor'],motion=p['motion'],kind='frame' if p['motion']=='fixed' else 'printed',color=col,offset=offset,vertices=len(a)//3));chunks.append(a);offset+=len(a)
for r in D['records']:
 t,_=lib.mesh(r['part']);t=t*.4@np.array(r['matrix']).reshape(3,3).T+np.array(r['pos'])*.4;n=r['record_id'];col=colors.get(n[2:],[85,92,94])
 if 'power' in n:col=[220,141,45]
 if 'idler' in n and r['part']=='10928.dat':col=[54,133,180]
 if 'port gear' in n or 'port shaft' in n:col=[54,133,180] if n.startswith('W') else [83,154,110]
 if 'HOLD roller' in n:col=[105,108,114]
 if r['part']=='2780.dat':col=[59,65,67]
 a=t.astype('<f4').reshape(-1);parts.append(dict(id=n,actor=r['actor'],motion=r['motion'],kind='pin' if r['part']=='2780.dat' else 'lego',color=col,offset=offset,vertices=len(a)//3));chunks.append(a);offset+=len(a)
vv=np.concatenate(chunks);data=dict(ports=D['ports'],parts=parts,geometry=base64.b64encode(gzip.compress(vv.tobytes())).decode(),actors=D['actors'],ends=ends,trace=trace,bounds=[vv.reshape(-1,3).min(0).tolist(),vv.reshape(-1,3).max(0).tolist()])
head='<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Two-actuator cam test rig</title><style>body{font:16px/1.5 system-ui;margin:0;background:#f6f5ee;color:#263e40}main{max-width:1400px;margin:auto;padding:20px}h1{margin:0}.tools{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:12px 0}button,select{font:inherit;padding:6px;border:1px solid #9badab;background:white;color:inherit;border-radius:4px}.scene{height:560px;position:relative}canvas{width:100%;height:100%;touch-action:none}.note{background:#eee4c9;padding:12px}.readout{font-weight:600}a{color:#176c76}</style><main><h1>Two-actuator cam test rig</h1><p>162 × 82 mm base · 22 printed parts · two input axles. The CAM CONTROL actuator moves the cam; its clutch drives the test actuator only after the bolt withdraws.</p><div class="tools"><label>Inspect <select id="state"><option value="hold0">Locked · end 0</option><option value="hold1">Locked · end 1</option><option value="write0">Unlocked · end 0</option><option value="write1">Unlocked · end 1</option></select></label><label>Focus <select id="focus"><option value="all">Full test rig</option><option value="K">Test actuator</option><option value="W">Cam-control actuator</option><option value="lock">Lock and cam</option></select></label><label><input type="checkbox" id="frame" checked>Show supports</label><label><input type="checkbox" id="labels" checked>Axle labels</label><button id="reset">Reset camera</button><button id="top">Top view</button></div><div class="tools"><label>Part <select id="part"><option value="">All parts</option></select></label><label><input type="checkbox" id="inspect">Inspect cam travel</label><input id="write" type="range" min="-4.35" max="4.325" step=".025" value="4.325" aria-label="Cam travel"></div><div class="tools"><button id="play">Play explanation</button><button id="restart">Restart</button><button id="explode">Explode lock parts</button><label>Sequence <input id="timeline" type="range" min="0" max="1000" value="0" aria-label="Animation timeline"></label><label><input id="cutaway" type="checkbox">Hide lock bridges</label><label><input id="pinlabels" type="checkbox" checked>Explain pins</label></div><p id="phase" class="note">Play to see HOLD → unlock → drive the test actuator → relock. The animation illustrates the intended sequence, not forces or measured timing.</p><p id="pinlegend">The yellow cam moves the bolt through its follower pin. The bolt guide is part of the front bridge; return-band anchors are part of the rear bridge.</p><div id="status" class="readout"></div><div class="scene"><canvas id="view"></canvas></div><p>Drag to rotate · Shift/right-drag to pan · Scroll to zoom.</p><h2>Bench test</h2><ol><li>With TEST DRIVE stopped, turn CAM CONTROL to withdraw the bolt and engage the clutch.</li><li>Turn TEST DRIVE slowly until the test carriage reaches an end. Reverse it and check travel to the other end.</li><li>At either end, return CAM CONTROL to HOLD. The clutch disconnects before the bolt seats. Turning TEST DRIVE should now leave the test carriage still.</li><li>Stop both inputs. Gently test carriage holding and cam-control backlash by hand; then unlock and repeat at the other end.</li></ol><p>Do not relock with the test carriage between its two end pockets. Keep the test drive set to the selected end while relocking.</p><p><a href="Print plate 1.stl">Print plate 1 · base</a> · <a href="Print plate 2.stl">Print plate 2 · components</a> · <a href="README.md">Build and test guide</a> · <a href="LEGO parts.csv">LEGO inventory</a></p><p class="note">Physical fit, band tension and loaded timing remain bench-test questions. No output-enable stage, storage power train or register output clutch is included. Three elastic bands: one per actuator and one for the locking bolt.</p><div id="error"></div></main>'
js=(ROOT/'viewer.js').read_text();(OUT/'Viewer.html').write_text(head+'<script type="application/json" id="data">'+json.dumps(data,separators=(',',':'))+'</script><script>'+js+'</script>')
# Render the assembled hold-0 pose with the same transforms used in checks.
from motion import transform
def trans(p,st):
 T=transform(p['actor'],p['motion'],st,D['actors']);return T[:3,:3],T[:3,3]
st={'K':ends['1'],'W':ends['1'],'E':ends['0']};ts=[];cs=[]
for p in parts:
 t=vv[p['offset']:p['offset']+p['vertices']*3].reshape(-1,3,3);R,dv=trans(p,st);ts.append(t@R.T+dv);r,g,b=p['color'];cs.append(np.full(len(t),0x02000000|(r<<16)|(g<<8)|b))
from band import lock_band
lt=lock_band(st['W']['q']).triangles;ts.append(lt);cs.append(np.full(len(lt),0x02a58148))
# Visible elastic return bands, one per actuator, from retained anchor positions.
for ac,off in D['actors'].items():
 a=np.array([36.96055698703205,-11.721496948498547]);pivot=np.array([14.50092459,-3.26189219]);b=np.array([21.987468724068282,-6.08176044140031]);ang=math.radians(st[ac]['b']);R=np.array([[math.cos(ang),-math.sin(ang)],[math.sin(ang),math.cos(ang)]]);b=pivot+R@(b-pivot);direction=math.atan2(b[1]-a[1],b[0]-a[0]);polys=[]
 for rad in [2.4,1.8]:polys.append(np.array([c+rad*np.array([math.cos(t),math.sin(t)]) for c,start in [(a,direction+math.pi/2),(b,direction-math.pi/2)] for t in np.linspace(start,start+math.pi,25)]))
 band=[];N=len(polys[0]);z=45.5
 for i in range(N):
  j=(i+1)%N
  for k,l in [(0,1)]:
   for zz in [-.6,.6]:band.extend([[np.r_[polys[0][i],z+zz],np.r_[polys[0][j],z+zz],np.r_[polys[1][i],z+zz]],[np.r_[polys[1][i],z+zz],np.r_[polys[0][j],z+zz],np.r_[polys[1][j],z+zz]]])
  for poly in polys:band.extend([[np.r_[poly[i],z-.6],np.r_[poly[j],z-.6],np.r_[poly[i],z+.6]],[np.r_[poly[i],z+.6],np.r_[poly[j],z-.6],np.r_[poly[j],z+.6]]])
 ts.append(np.array(band)@(np.diag([-1,1,-1]) if ac=='W' else np.eye(3))+off);cs.append(np.full(len(band),0x029e8850))
class Scene:
 source=OUT/'Assembly manifest.json';root='Two-actuator cam test rig';missing={};warnings=[]
 def mesh(self,root=None):return np.concatenate(ts)*2.5,np.concatenate(cs)
 def rgb(self,c):return[(int(c)>>16)&255,(int(c)>>8)&255,int(c)&255]
render(Scene(),OUT/'Assembly.png',azimuth=130,elevation=48,width=1600,height=1000,supersample=1)
print('Viewer and assembly image saved',flush=True)
