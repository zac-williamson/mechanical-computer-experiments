import os
from pathlib import Path
import sys,json,math,copy,re,gzip,base64,collections
import numpy as np
import manifold3d as m
import trimesh
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'Sources/Inputs/multiplexer';OUT=ROOT;OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(Path(__file__).resolve().parent))
from render_ldraw import LDraw
LIB=LDraw(str(Path(os.environ.get('LDRAW_PATH','/Applications/Studio 2.0/ldraw'))/'parts/23948.dat'))
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

# Snap below manufacturing resolution before STL export to avoid float32 slivers.
def mesh(s):
 a=s.to_mesh64();t=trimesh.Trimesh(np.round(np.asarray(a.vert_properties)[:,:3],5),np.asarray(a.tri_verts),process=True)
 t.update_faces(t.nondegenerate_faces());t.update_faces(t.unique_faces());t.remove_unreferenced_vertices();return t
Aold=json.loads((ROOT/'Sources/Inputs/old-adder.json').read_text()); mux=A
mods={'P':(np.array([0.,0,0]),1),'X':(np.array([76.,0,32]),1),'C':(np.array([76.,0,0]),-1),'S':(np.array([0.,-16,-32]),-1)}
actors={k:d for k,(d,sg) in mods.items()}
keep=['U015','U022','U185','reaction-axle','reaction-retainer-14','reaction-retainer-50','reaction-spacer-24','reaction-spacer-40','pivot-axle-with-stop','U017','U019','U032','SECOND-GUIDE','SECOND-GUIDE-BUSH-R','SECOND-GUIDE-BUSH-L','pivot-front-half-bush','pivot-spacer-22','pivot-hub-rear-bush','pivot-rear-retainer','reaction-rear-extra-half-bush','selector-right-retainer','Carriage joining pin top','Carriage joining pin base','C-shaft','O-shaft','L072','L097','L099','L102','L069','L105']
feet={'Front actuator bridge':[(-28,8),(-28,16),(36,8),(36,16)],'Rear bridge and band anchor':[(-8,51),(8,51),(36,43),(36,51)],'Front bottom guide':[(-8,8),(8,8)],'Bearing wall X26':[(26,24),(26,40)],'Bearing wall X-28 three holes':[(-28,24),(-28,40)]}
for ac,(d,sg) in mods.items():
 R=np.diag([sg,1,sg]);od=np.array(next(v['offset'] for v in Aold['modules'] if v['id']==ac))
 for n in list(feet)+['Left carriage half','Right carriage half','Direct lever and band cleat']:
  s=load(n).transform(np.column_stack([R,d]));motion='carriage' if 'carriage' in n else 'rocker' if 'lever' in n else 'fixed'
  if ac=='S' and n in feet:
   for xx,zz in feet[n]:
    pp=R@np.array([xx,28,zz])+d;s=s+cyl(pp,3.4,17,'y')
   for xx,zz in feet[n]:
    pp=R@np.array([xx,28,zz])+d
    f=s^box([pp[0]-3.8,4,pp[2]-3.8],[pp[0]+3.8,12,pp[2]+3.8]);s=s+hull([f,f.translate([0,16,0])])
  for x,z in feet.get(n,[]):
   p=R@np.array([x,28,z])+d;p[1]=28;s=pin_cut(s,p,'y');native(ac+' mount '+n+' '+str(x)+' '+str(z),'2780.dat',p,np.array(old['FRAME-PIN-1']['matrix']).reshape(3,3));mounts.append((p[0],p[2]))
  if ac=='S' and n in feet:
   s=union([c for c in s.decompose() if c.volume()>0])^box([-200,-200,-200],[200,28,200])
  put(ac+' '+n,s,np.array(orient[n])@R.T,ac,motion)
 rs=[]
 for v in mux['records']:
  if v['record_id'] not in keep:continue
  r=copy.deepcopy(v);rid=r['record_id'];pos=np.array(r['pos'])*.4
  if ac=='P' and rid=='C-shaft':r['part']='3737.dat';pos=[4,10.2,32]
  if ac=='X' and rid=='O-shaft':r['part']='60485.dat';pos=[4.4,10.2,0]
  if ac=='P' and rid=='O-shaft':r['part']='3708.dat';pos=[-8.6,10.2,0]
  if ac=='C' and rid=='C-shaft':r['part']='3708.dat';pos=[12,10.2,32]
  if ac=='C' and rid=='O-shaft':r['part']='60485.dat';pos=[-8.4,10.2,0]
  if ac=='S' and rid=='C-shaft':r['part']='3737.dat';pos=[-4,10.2,32]
  if ac=='S' and rid=='O-shaft':r['part']='60485.dat';pos=[-8,10.2,0]
  if rid in ['U017','SECOND-GUIDE']:r['part']='60485.dat'
  r['pos']=((R@np.array(pos)+d)/.4).tolist();r['matrix']=(R@np.array(r['matrix']).reshape(3,3)).reshape(-1).tolist();r['record_id']=ac+' '+rid;r['actor']=ac;records.append(r)
 for v in Aold['records']:
  if v.get('module')!=ac:continue
  n=v['record_id'].split(' ',1)[1]
  if not n.startswith(('R-P','Power gear','Power idler','Carry left','Carry right')):continue
  if ac=='C' and (n=='Carry right input' or n in ['Power gear collar 38.4','Power gear collar 26.4']):continue
  if ac in ['P','S'] and n=='Power gear collar 38.4':continue
  r=copy.deepcopy(v);pos=np.array(r['pos'])*.4-od
  if ac=='P' and n=='R-P-shaft':r['part']='3708.dat';pos=[8,-5.8,0]
  if ac=='S' and n=='R-P-shaft':r['part']='23948.dat';pos=[12,-5.8,0]
  if ac=='C' and n=='Carry left input':r['part']='60485.dat';pos=[-20,-5.8,0]
  if ac in ['P','S'] and n in ['R-P-right','R-P-idler-back']:pos[0]=34.4 if ac=='P' else 32.4
  r['pos']=((R@pos+d)/.4).tolist();r['matrix']=(R@np.array(r['matrix']).reshape(3,3)).reshape(-1).tolist();r['record_id']=ac+' '+n;r['actor']=ac;r['motion']='fixed';records.append(r)
# Completed coaxial B' and selector distribution connections.
for name,pos in [('Bprime',[44.2,10.2,32]),('Carry selector',[15.8,10.2,-32]),('Cin selector',[44.2,-5.8,-64])]:native(name+' connector','59443.dat',pos)
native('Carry selector extension','60485.dat',[-20.4,10.2,-32],np.eye(3));native('Cin selector extension','23948.dat',[88.4,-5.8,-64],np.eye(3))
# Left routing: P -> NOT P for the mirrored carry selector, then P for mirrored sum data.
left=[(10.2,0,'4019.dat'),(10.2,-12,'10928.dat'),(10.2,-20,'10928.dat'),(10.2,-32,'4019.dat'),(-1.8,-32,'10928.dat'),(-9.8,-32,'10928.dat'),(-21.8,-32,'4019.dat')]
for i,(y,z,part) in enumerate(left):
 native('P route gear '+str(i),part,[-48.6,y,z])
 if i not in [0,3,6]:native('P route axle '+str(i),'4519.dat',[-44.6,y,z],np.eye(3))
 native('P route outer bush '+str(i),'32123a.dat',[-54.6,y,z])
 if i not in [0,3,6]:native('P route inner bush '+str(i),'32123a.dat',[-42.6,y,z])
# Cin -> NOT Cin rail. Four following meshes keep NOT Cin at the carry data input.
right=[(-21.8,-64),(-5.8,-64),(-5.8,-48),(-5.8,-32),(-5.8,-16),(-5.8,0)]
for i,(y,z) in enumerate(right):
 native('Cin route gear '+str(i),'4019.dat',[124,y,z])
 if i not in [1,5]:native('Cin route axle '+str(i),'3705.dat' if i==0 else '4519.dat',[120,y,z],np.eye(3))
 for x in [118,130]:native('Cin route bush '+str(i)+' '+str(x),'32123a.dat',[x,y,z])
# Flat printed bearing plates. All holes are parallel to X; feet grow only away from the bed face.
def support(name,x,holes,footz):
 s=union([box([x-2,y,z-4],[x+2,28,z+4])+cyl([x,y,z],4,4,'x') for y,z in holes])
 s=s+box([x-2,20,min(z for y,z in holes)-4],[x+2,28,max(z for y,z in holes)+4])
 for z in footz:
  px=x+3.5;s=s+box([x-2,20,z-5.5],[x+9,28,z+5.5]);s=pin_cut(s,[px,28,z]);pin(name+' mounting '+str(z),[px,28,z]);mounts.append((px,z))
 for z in footz:s=pin_cut(s,[x+3.5,28,z])
 for y,z in holes:s=s-cyl([x,y,z],2.65,4.4,'x')
 put(name,s,FX,bores=[[x,y,z] for y,z in holes])
support('P distribution and A bearing',-38.6,[(y,z) for y,z,p in left]+[(-5.8,0)],[-28,-2])
support('Cin distribution bearing',114,right+[(10.2,-32)],[-64,-32,0])
support('P and sum data bearing',26.2,[(10.2,0),(2.2,-np.sqrt(80)),(-5.8,-32),(-21.8,-32),(10.2,-32)],[-28,-2])
support('Shared A carry and Bprime bearing',49.8,[(-5.8,0),(10.2,0),(2.2,np.sqrt(80)),(-5.8,32),(10.2,32)],[-4,16,28])
support('Carry and X output bearing',102.2,[(10.2,0),(-5.8,0),(10.2,32),(2.2,32-np.sqrt(80))],[4,20,28])
support('Sum left output and idler bearing',-26,[(-5.8,-32),(-13.8,-32+np.sqrt(80))],[-36,-19.056])
support('X right data bearing',120,[(-5.8,32)],[20,28])
# Additional shaft collars at bearing ends, not in the shared A gear gap.
for n,p in [('P output',[34,10.2,0]),('Sum data spacer',[-42.6,-21.8,-32]),('Sum output',[34,-5.8,-32]),('Carry output',[110,10.2,0]),('Carry selector end',[110,10.2,-32])]:native(n+' collar','32123a.dat',p)
assert len(set(mounts))==len(mounts),'duplicate mounting pins'
base=box([-44,28,-89],[132,36.2,89])
for x,z in mounts:base=pin_cut(base,[x,28,z],length=20)
put('Base',base,FY)
D=dict(name='Single-level compact adder prototype',records=records,prints=prints,actors={k:v.tolist() for k,v in actors.items()},orientations={k:sg for k,(d,sg) in mods.items()},mounts=mounts,base_mm=[176,178,8.2],status='Review draft — two travel clearances unresolved; not ready to print',ports={'A':[-40,-5.8,0],'B':[128,-5.8,32],'SU':[116,10.2,64],'Cin':[136,-21.8,-64],'Sum':[44,-5.8,-32],'Cout':[120.4,10.2,0]})
(OUT/'Assembly manifest.json').write_text(json.dumps(D,indent=2));print('Built',len(prints),'parts',len(records),'native',len(mounts),'pins',flush=True)
