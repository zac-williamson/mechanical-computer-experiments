from pathlib import Path
import sys,json,math,copy,re,gzip,base64,collections
import numpy as np
import manifold3d as m
import trimesh
ROOT=Path(__file__).resolve().parent;import os
SRC=ROOT/'Inputs';OUT=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(ROOT))
from render_ldraw import LDraw
LIB=LDraw(str(Path(os.environ.get('LDRAW_DIR','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'))
A=json.loads((SRC/'Assembly manifest.json').read_text());old={r['record_id']:r for r in A['records']}
orient={r['part']:np.array(r['rotation']) for r in json.loads((SRC/'Print checks.json').read_text())['parts']}
RAX=np.array([[0,0,1],[0,1,0],[-1,0,0]]);RY=np.array([[1,0,0],[0,0,1],[0,-1,0]])
FX=np.array([[0,0,1],[0,-1,0],[1,0,0]]);FZ=np.diag([1,-1,-1]);FY=np.array([[1,0,0],[0,0,1],[0,-1,0]])
actors={'K':np.array([0.,0,0]),'W':np.array([-80.,0,32.]),'E':np.array([80.,0,0])}
solids={};prints=[];records=[];mounts=[]
def box(lo,hi):return m.Manifold.cube((np.array(hi)-lo).tolist()).translate(lo)
def cyl(c,r,l,axis='z',segments=64):
 s=m.Manifold.cylinder(l,r,circular_segments=segments,center=True)
 if axis=='x':s=s.rotate((0,90,0))
 if axis=='y':s=s.rotate((90,0,0))
 return s.translate(c)
def union(ss):return m.Manifold.batch_boolean(list(ss),m.OpType.Add)
def hull(ss):return m.Manifold.batch_hull(list(ss))
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(np.asarray(a.vert_properties)[:,:3],np.asarray(a.tri_verts),process=True)
def load(name):
 t=trimesh.load(SRC/(name+'.stl'));return m.Manifold(m.Mesh64(np.asarray(t.vertices),np.asarray(t.faces,dtype=np.uint64)))
def native(name,part,p,R=RAX,actor='fixed',motion='fixed',role=''):
 r=dict(record_id=name,part=part,pos=(np.array(p)/.4).tolist(),matrix=np.array(R).reshape(-1).tolist(),actor=actor,motion=motion,role=role);records.append(r);return r
def copy_native(actor,rid,part=None,pos=None,motion=None):
 r=copy.deepcopy(old[rid]);r['record_id']=actor+' '+rid;r['actor']=actor
 r['pos']=(np.array(r['pos'])+actors[actor]/.4).tolist()
 if part:r['part']=part
 if pos is not None:r['pos']=(np.array(pos)/.4).tolist()
 if motion:r['motion']=motion
 records.append(r);return r
def put(name,s,rot,actor='fixed',motion='fixed',bores=None,sliding=None):
 assert s.status()==m.Error.NoError,(name,s.status());s=s.simplify(.0001)
 assert len(s.decompose())==1,(name,len(s.decompose()))
 solids[name]=s;t=mesh(s);assert t.is_watertight and t.is_winding_consistent,name;t.export(OUT/(name+'.stl'))
 prints.append(dict(id=name,actor=actor,motion=motion,rotation=np.asarray(rot).tolist(),bores=bores or [],sliding=sliding or [],path=name+'.stl'))
 return s
def pin_cut(s,p,axis='y',length=17):
 s=s-cyl(p,2.46,length,axis)
 # collar recess at mounting joint
 return s-cyl(p,3.3,1.2,axis)
def pin(name,p,axis='y',actor='fixed',motion='fixed'):
 R=np.array(old['FRAME-PIN-1']['matrix']).reshape(3,3) if axis=='y' else np.eye(3) if axis=='x' else RAX
 native(name,'2780.dat',p,R,actor,motion)
def wall(name,x,holes,zrange,pinz,actor='fixed'):
 # Broad, connected plate with round crowns and flat print face.
 s=union([box([x-3.8,y,z-4.3],[x+3.8,28,z+4.3])+cyl([x,y,z],4.3,7.6,'x') for y,z in holes])
 s=s+box([x-3.8,20,zrange[0]],[x+3.8,28,zrange[1]])
 s=s-box([x-4,20,zrange[1]],[x+4,29,zrange[1]+5])
 for y,z in holes:s=s-cyl([x,y,z],2.65,9,'x')
 for z in pinz:
  p=[x,28,z];s=pin_cut(s,p);mounts.append((x,z));pin(name+' mounting '+str(z),p)
 return s
# Copy the proven actuator, with new storage lock and write-link attachments below.
keep=['U015','U022','U185','reaction-axle','reaction-retainer-14','reaction-retainer-50','reaction-spacer-24','reaction-spacer-40','pivot-axle-with-stop','U017','U019','U032','SECOND-GUIDE','SECOND-GUIDE-BUSH-R','SECOND-GUIDE-BUSH-L','pivot-front-half-bush','pivot-spacer-22','pivot-hub-rear-bush','pivot-rear-retainer','reaction-rear-extra-half-bush','selector-right-retainer','Carriage joining pin top','Carriage joining pin base']
fixedparts=['Front actuator bridge','Front bottom guide','Bearing wall X26','Bearing wall X-28 three holes']
actuatormount=[(-28,24),(-28,40),(26,24),(26,40),(-8,8),(8,8),(-8,51),(8,51),(-28,8),(-28,16),(36,8),(36,16),(36,43),(36,51)]
for ac,d in actors.items():
 for rid in keep:copy_native(ac,rid)
 if ac!='K':copy_native(ac,'U072')
 copy_native(ac,'C-shaft',part='3737.dat',pos=d+[4.8 if ac=='E' else 0,10.2,32])
 for name in fixedparts:
  put(ac+' '+name,load(name).translate(d),orient[name],ac,'fixed')
 for x,z in actuatormount:
  p=d+[x,28,z];mounts.append((float(p[0]),float(p[2])));pin(ac+' frame pin '+str(x)+' '+str(z),p)
 for name in ['Left carriage half','Right carriage half']:
  s=load(name)
  if ac=='K':
   for xc in [-11.325,-2.65]:s=s-box([xc-2.05,-22.3,29.5],[xc+2.05,-13.7,32.4])
  if ac=='W':
   if name=='Left carriage half':s=pin_cut(s,[-15.6,-18.6,34],'x')
  put(ac+' '+name,s.translate(d),orient[name],ac,'carriage')
 put(ac+' Direct lever and band cleat',load('Direct lever and band cleat').translate(d),orient['Direct lever and band cleat'],ac,'rocker')
 rear=load('Rear bridge and band anchor')
 put(ac+' Rear bridge and band anchor',rear.translate(d),FZ,ac,'fixed')
 # State ring and its keyed hub. W/E use one gear and a neutral spacer.
 for rid in ['L097','L099','L105','L069']:copy_native(ac,rid)
 if ac!='W':copy_native(ac,'L102')
 if ac in ['K','W']:copy_native(ac,'L072')
 if ac=='W':native(ac+' neutral spacer','3713.dat',d+[-16,10.2,0],actor=ac)
 elif ac=='E':native(ac+' neutral spacer','3713.dat',d+[16,10.2,0],actor=ac,role='Replaces unused right clutch gear; neutral dog position')
 copy_native(ac,'O-shaft',part='3737.dat',pos=d+[0,10.2,0])
 if ac=='K':copy_native(ac,'O-left')
 if ac!='K':copy_native(ac,'O-right')
 # W right and E left retention supplied by the native joining connector.
 if ac=='W':pass
 if ac=='K':pass
# Native keyed connectors join end-to-end shafts without overlapping their ends.
for x,z in [(-40,32),(40,0)]:native('Shaft connector '+str(x),'6538a.dat',[x,10.2,z],role='2L keyed connector; also outer bearing retainer')
# Storage power train from the study.
y=10.2-math.sqrt(80)
native('K power shaft','3708.dat',[0,10.2,-24],np.eye(3),role='Continuous CW power; 12L')
for x in [-16,16]:native('K power gear '+str(x),'94925.dat',[x,10.2,-24])
for x in [-34,-22,-10,10,22,34]:native('K power collar '+str(x),'32123a.dat',[x,10.2,-24])
for name,x,yy,z in [('K first reversing idler',-16,y,-16),('K second reversing idler',-16,y,-8),('K direct idler',16,10.2,-12)]:
 native(name,'10928.dat',[x,yy,z]);native(name+' axle','32073.dat',[x if x<0 else 20,yy,z],np.eye(3))
 for bx in ([-34,-22,-10,2] if x<0 else [10,22,34]):native(name+' collar '+str(bx),'32123a.dat',[bx,yy,z])
for x in [-28,28]:
 holes=[(10.2,0),(10.2,-24)]+([(y,-16),(y,-8)] if x<0 else [(10.2,-12)])
 s=wall('K outer bearing '+str(x),x,holes,[-32,4],[-24,-8,0]);put('K outer bearing '+str(x),s,FX,bores=[[x,yy,z] for yy,z in holes])
for x,holes,pinz in [(-4,[(y,-16),(y,-8)],[-24,-8]),(4,[(10.2,-12)],[-24,-12])]:
 s=wall('K inner bearing '+str(x),x,holes,[-28,-3.7 if x<0 else -7.7],pinz)
 s=s-cyl([x,10.2,0],7.6,9,'x')
 put('K inner bearing '+str(x),s,FX,bores=[[x,yy,z] for yy,z in holes])
# Direct 16T meshes: data drives W hub; W clutch gear drives K selector.
for ac in ['W','E']:
 d=actors[ac];cx=d[0]+(16 if ac=='W' else -16);mainz=d[2]
 native(ac+' port gear','94925.dat',[cx,10.2,mainz-16],actor=ac)
 native(ac+' port shaft','32073.dat',[d[0]+16 if ac=='W' else d[0]-10,10.2,mainz-16],np.eye(3),ac,role='D input gear shaft' if ac=='W' else 'BUS_OUT')
 for bx in ([d[0]-2,d[0]+10,d[0]+22,d[0]+34] if ac=='W' else [d[0]-22,d[0]-10,d[0]+2]):
  native(ac+' port collar '+str(bx),'32123a.dat',[bx,10.2,mainz-16],actor=ac)
 for xx in [-24,28]:
  holes=[(10.2,mainz)]+([(10.2,mainz-16)] if (ac=='W' and xx>0) or (ac=='E' and xx<0) else [])
  x=d[0]+xx;name=ac+(' left bearing' if xx<0 else ' right bearing');s=wall(name,x,holes,[mainz-20,mainz+4],[mainz-16,mainz])
  if xx==-24:s=s-box([x-.2,-20,mainz-25],[x+4,20,mainz+10])
  put(name,s,FX,ac,bores=[[x,yy,z] for yy,z in holes])
 x=d[0]+(4 if ac=='W' else -4);holes=[(10.2,mainz-16)];name=ac+' port bearing';s=wall(name,x,holes,[mainz-20,mainz-11.7],[mainz-16]);put(name,s,FX,ac,bores=[[x,yy,z] for yy,z in holes])
# Low transverse bolt. The nose prints on Z32; running faces are vertical.
body=box([-13,-22,12],[-1,-14,23.5]);nose=box([-8.8,-22,29.5],[-5.2,-14,32]);bolt=body+nose+hull([box([-13,-22,23.3],[-1,-14,23.5]),box([-8.8,-22,29.3],[-5.2,-14,29.5])])
bolt=pin_cut(bolt,[-7,-22,18],'y')
# Broad band saddle, with retaining shoulders; no printed spring or snap feature.
bolt=bolt+box([-9,-28.5,27.5],[-5,-22,29.5])+box([-10.5,-29.5,27],[-3.5,-28.5,30])
put('HOLD side bolt',bolt,FZ,'lock','bolt',sliding=['X and Y guide faces are vertical when nose Z32 is on bed; nose has pocket-floor clearance'])
pin('HOLD cam follower',[-7,-22,18],'y','lock','bolt')
# Guide, cap and keeper are integral with the front bridge. All remain
# behind its Z19.799999237060547 axle-bore bed face, so the original print orientation works.
front='K Front actuator bridge'
guide=box([-17,-13.7,8],[3,-10.7,19.799999237060547])+box([-17,-22.3,8],[-13.3,-12,19.799999237060547])+box([-.7,-22.3,8],[3,-12,19.799999237060547])
cap=box([-17,-24.5,8],[3,-22.3,19.799999237060547])-box([-11,-25,10],[-3,-22,20])
keeper=box([-12,-31.3,7],[-2,-24.5,9.7])+box([-12,-33.3,8],[-2,-31.3,11.5])
solids[front]=(solids[front]+guide+cap+keeper).simplify(.0001)
assert len(solids[front].decompose())==1
mesh(solids[front]).export(OUT/(front+'.stl'))
# Band anchors grow forward from the existing rear bridge, never past its
# Z54.79999923706055 printing face. The band endpoints and bolt stroke remain unchanged.
rear='K Rear bridge and band anchor'
ears=[box([-19,-27.5,48.20000076293945],[5,-23.8,54.79999923706055])]
for x in [-17,3]:
 ears.append(box([x-2,-27.5,37],[x+2,-23.8,54.79999923706055]))
 ears.append(box([x-4,-29,37],[x+4,-27.5,39]))
from band import lock_band
bandcuts=[]
for q in np.linspace(-4.35,4.325,13):
 bt=lock_band(q,.15);bandcuts.append(m.Manifold(m.Mesh64(np.asarray(bt.vertices),np.asarray(bt.faces,dtype=np.uint64))))
solids[rear]=(solids[rear]+(union(ears)-union(bandcuts))).simplify(.0001)
assert len(solids[rear].decompose())==1
mesh(solids[rear]).export(OUT/(rear+'.stl'))
# One narrow diagonal link with a closed cam slot. HOLD is 2.5 mm inside the flat.
t,_=LIB.mesh('2780.dat')
# One horizontal friction pin enters the end face of W's carriage roof.
root=box([-60.4,-23,-6.2],[-52.4,-14.2,2.2])
root=pin_cut(root,[-60.4,-18.6,-2],'x')
pin('Write cam side pin',[-60.4,-18.6,-2],'x','link','carriage-global')
neck=box([-30,-31,9],[-28,-25,17])
cam=root+hull([box([-54,-22.6,-5.8],[-52.4,-14.6,1.8]),neck])+hull([neck,box([-14,-31,13.8],[-10,-25,22])])
cam=cam+box([-18,-31,10],[4,-25,22])
path=[[-14,18],[-8.85,18],[-6.5,15.2],[0,15.2]]
track=union([hull([cyl([a[0],-28,a[1]],2.7,8,'y'),cyl([b[0],-28,b[1]],2.7,8,'y')]) for a,b in zip(path,path[1:])]);cam=cam-track
cam=cam-box([-100,-40,22],[10,-20,30])
cam=pin_cut(cam,[-60.4,-18.6,-2],'x')
put('Write diagonal cam',cam,np.array([[1,0,0],[0,0,-1],[0,1,0]]),'link','carriage-global',sliding=['Cam slot walls print vertically; no axle bores'])
# Rigid 180-degree rotation about Y keeps genuine LEGO worm handedness.
newactors={'K':np.array([0.,0,0]),'W':np.array([-76.,0,32]),'E':np.array([76.,0,0])}
turn=np.diag([-1.,1.,-1.])
for p in prints:
 ac=p['actor']
 if ac not in ['W','E']:continue
 R=turn if ac=='W' else np.eye(3);t=newactors[ac]-R@actors[ac]
 solids[p['id']]=solids[p['id']].transform(np.column_stack([R,t]));mesh(solids[p['id']]).export(OUT/p['path'])
 p['rotation']=(np.array(p['rotation'])@R.T).tolist();p['bores']=[(R@np.array(c)+t).tolist() for c in p['bores']]
for r in records:
 ac=r['record_id'].split(' ')[0] if r['actor']=='fixed' else r['actor']
 if ac not in ['W','E']:continue
 R=turn if ac=='W' else np.eye(3);t=newactors[ac]-R@actors[ac]
 r['pos']=((R@(np.array(r['pos'])*.4)+t)/.4).tolist();r['matrix']=(R@np.array(r['matrix']).reshape(3,3)).reshape(-1).tolist()
 if r['record_id']=='W O-shaft' or r['record_id']=='E O-shaft':r['part']='60485.dat'
for r in records:
 if r['record_id']=='W C-shaft':
  r['part']='60485.dat';r['pos']=(np.array([-76.8,10.2,0])/.4).tolist()
# Replace mount positions from actual transformed frame-pin centres.
mounts=[(r['pos'][0]*.4,r['pos'][2]*.4) for r in records if r['part']=='2780.dat' and abs(r['pos'][1]*.4-28)<1e-5]
actors=newactors
# Base: all pin holes through, nominal tips 0.2 mm above underside.
assert len(set(mounts))==len(mounts),'Duplicate base pins'
base=box([-116,28,-40],[120,36.2,60])
for x,z in mounts:base=pin_cut(base,[x,28,z],length=20)
put('Base',base,FY)
# Apply checked tooth phases; keyed idler shafts and collars follow their gears.
phases={'K direct idler':22.5,'K first reversing idler':32.0625,'K second reversing idler':35.4375,'K L102':11.3125,'W port gear':11.25,'E port gear':11.25}
for r in records:
 name=r['record_id'];ph=next((v for k,v in phases.items() if name==k or name.startswith(k+' axle') or name.startswith(k+' collar')),0)
 if name in ['E port shaft','W port shaft'] or name.startswith('E port collar ') or name.startswith('W port collar '):ph=11.25
 if ph:
  a=math.radians(ph);c,si=math.cos(a),math.sin(a);rx=np.array([[1,0,0],[0,c,-si],[0,si,c]])
  r['matrix']=(rx@np.array(r['matrix']).reshape(3,3)).reshape(-1).tolist();r['phase_deg']=ph
 if name.endswith('C-shaft'):r['role']=('9L offset write-control axle' if name=='W C-shaft' else '10L selector/control axle; see port coordinates')
 if name.endswith('O-shaft'):r['role']=('10L' if name=='K O-shaft' else '9L')+' clutch hub shaft'
manifest=dict(name='Compact lock register prototype',status='Geometric prototype; physical fit and timing untested',records=records,prints=prints,actors={k:v.tolist() for k,v in actors.items()},mounts=mounts,stroke=[-4.35,4.325],hold={'bolt_axis':'Z','locked_tip_Z':32,'lift_mm':2.8,'hold_dwell_mm':2.5,'release_complete_W_q':.5,'W_write_endpoint':4.325,'W_hold_endpoint':-4.35,'Q_is_inverted':True,'return_band':'Symmetric loop from moving saddle to two rear-bridge ears'},ports={'D':[-112,10.2,48],'P':[-48,10.2,-24],'Q':[-40,10.2,0],'BUS_OUT':[86,10.2,-16],'W':[-112.8,10.2,0],'OE':[120.8,10.2,32]})
(OUT/'Assembly manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(OUT/'LEGO parts.csv').write_text('Part,Quantity\n'+''.join(f'{p},{n}\n' for p,n in sorted(collections.Counter(r['part'] for r in records).items())))
print('Built',len(prints),'printed parts,',len(records),'LEGO components,',len(mounts),'base pins',flush=True)
