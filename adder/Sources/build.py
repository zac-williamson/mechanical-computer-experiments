import os
from pathlib import Path
import sys,json,math,copy,re,gzip,base64,collections
import numpy as np
import manifold3d as m
import trimesh
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT.parent/'multiplexer';OUT=ROOT;OUT.mkdir(exist_ok=True)
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

pitch=76.0
mods={'X':np.array([0.,0,32]),'P':np.array([pitch,0,48]),'C':np.array([pitch+72,0,16]),'S':np.array([pitch+144,0,16])}
actors=mods
feet={'Front actuator bridge':[(-36,8),(-36,16),(36,8),(36,16)],'Rear bridge and band anchor':[(-8,51),(8,51),(36,43),(36,51)],'Front bottom guide':[(-8,8),(8,8)]}
keep=['U015','U022','U185','reaction-axle','reaction-retainer-14','reaction-retainer-50','reaction-spacer-24','reaction-spacer-40','pivot-axle-with-stop','U017','U019','U032','SECOND-GUIDE','SECOND-GUIDE-BUSH-R','SECOND-GUIDE-BUSH-L','pivot-front-half-bush','pivot-hub-rear-bush','pivot-rear-retainer','reaction-rear-extra-half-bush','selector-right-retainer','Carriage joining pin top','Carriage joining pin base','L072','L097','L099','L102','L069','L105']
passive=['U017','U019','U032','SECOND-GUIDE','SECOND-GUIDE-BUSH-R','SECOND-GUIDE-BUSH-L','Carriage joining pin base','L072','L097','L099','L102','L069','L105']
for ac,d in mods.items():
 for n in list(feet)+['Left carriage half','Right carriage half','Direct lever and band cleat']:
  if ac=='S' and n not in ['Front bottom guide','Left carriage half','Right carriage half']:continue
  s=load(n)
  if ac in ['X','P','C'] and n=='Left carriage half':
   # Clearance for the fixed pivot axle over the entire carriage stroke.
   # A 2.9 mm radial envelope leaves 0.5 mm beyond a 4.8 mm axle.
   # This relieves the inside roof corner, away from the lever contact pads.
   px,py=14.500924592586399,-3.261892187851191
   s-=hull([cyl([px-q,py,36],2.9,40) for q in [-4.35,4.325]])
  if n=='Front actuator bridge':
   leg=s^box([-100,-100,-100],[-24,100,100])
   s=(s-leg)+leg.translate([-8,0,0])+box([-32.1,-12,14],[-23.9,-6,19.799999])
  if ac=='S' and 'carriage' in n:s=s^box([-100,5,-100],[100,100,100])
  if ac=='C' and 'carriage' in n:
   # Fill the retired front-guide bore before forming a smooth gear relief.
   bx=-11.8 if n=='Left carriage half' else 11.8
   s+=cyl([bx,10.2,24],4.3,7.6,'x')
   if n=='Left carriage half':s+=cyl([bx,10.2,16],5.5,7.6,'x')
   # The bottom rail and rear guide constrain roll; clear the new data feed.
   s-=cyl([0,10.2,16],2.9,120,'x')
   s-=cyl([16,10.2,16],9.2,18,'x')
  if ac in ['C','S'] and n=='Left carriage half':
   # Flat lap joint, two ordinary friction pins; no snap fits or dovetails.
   pad=box([-15.6,15.6,17],[2,25.5,25])
   s=(s-box([-15.6,15.6,25],[2,25.5,34]))+pad
   for xx in [-10.8,-2.8]:s=pin_cut(s,[xx,20.55,25],'z',17)
  if ac=='P' and n=='Front actuator bridge':
   leg=s^box([-100,-100,-100],[-24,100,100]);s=(s-leg)+leg.translate([4,0,0])
  if ac=='C' and n=='Front actuator bridge':
   foot=s^box([-100,20,-100],[100,100,100]);s=(s-foot)+foot.translate([0,0,4])
   s=s.translate([0,0,-8])
  s=s.translate(d.tolist())
  motion='carriage' if 'carriage' in n else 'rocker' if 'lever' in n else 'fixed'
  for x,z in feet.get(n,[]):
   if ac=='P' and n=='Front actuator bridge' and x<0:x+=4
   p=np.array([x,28,z-(4 if ac=='C' and n=='Front actuator bridge' else 0)])+d;s=pin_cut(s,p)
   native(ac+' mount '+n+' '+str(x)+' '+str(z),'2780.dat',p,np.array(old['FRAME-PIN-1']['matrix']).reshape(3,3));mounts.append((p[0],p[2]))
  put(ac+' '+n,s,np.array(orient[n]),ac,motion)
 for r0 in A['records']:
  rid=r0['record_id']
  if ac=='C' and rid=='U185':continue
  if ac=='C' and rid in ['SECOND-GUIDE','SECOND-GUIDE-BUSH-R','SECOND-GUIDE-BUSH-L']:continue
  if (ac in ['P','C','S'] and rid=='L105') or (ac in ['P','C','S'] and rid=='L069'):continue
  if rid not in (passive if ac=='S' else keep) or (ac=='X' and rid=='L105'):continue
  r=copy.deepcopy(r0);r['record_id']=ac+' '+rid;r['actor']=ac;r['pos']=(np.array(r['pos'])+d/.4).tolist()
  if rid in ['U017','SECOND-GUIDE']:r['part']='3707.dat';r['pos'][0]=d[0]/.4
  if rid in ['U019','SECOND-GUIDE-BUSH-R']:r['pos'][0]=(d[0]+30)/.4
  if rid in ['U032','SECOND-GUIDE-BUSH-L']:r['pos'][0]=(d[0]-30)/.4
  if rid=='U185':r['pos'][0]=(d[0]-30)/.4
  if rid=='selector-right-retainer':r['pos'][0]=(d[0]+30)/.4
  if ac=='C' and rid in ['reaction-axle','pivot-axle-with-stop']:
   r['part']='44294.dat';r['pos'][2]-=4/.4
  if ac=='C' and rid in ['reaction-retainer-14','pivot-front-half-bush']:r['pos'][2]-=8/.4
  if (ac in ['P','C','S'] and rid=='L105') or (ac=='S' and rid=='L069'):r['pos'][0]=(d[0]+(-20 if rid=='L105' else 20))/.4
  records.append(r)
 for xx in [-24,24]:
  holes=[(10.2,d[2]+z) for z in ([32,40] if ac=='C' else [24,32,40])]
  name=ac+' guide bearing '+str(xx)
  sh=wall(name,d[0]+xx,holes,[d[2]+20,d[2]+52],[d[2]+40,d[2]+48])
  if ac in ['C','S']:sh-=box([d[0]+xx-5,18,d[2]+24.5],[d[0]+xx+5,25,d[2]+28.5])
  put(name,sh,FX,ac)

# One P-driven actuator moves both clutches through a pinned straight tie.
c=mods['C'];ss=mods['S']
link=box([c[0]-12,18.5,41],[ss[0]-4,24.5,44])
for d in [c,ss]:
 link+=box([d[0]-15.6,15.6,41],[d[0]+2,25.5,49])
 for xx in [-10.8,-2.8]:
  pp=[d[0]+xx,20.55,41];link=pin_cut(link,pp,'z',17);pin('Shared carriage coupling pin '+str(pp),pp,'z','C','carriage')
put('Shared carry and sum carriage tie',link,np.eye(3),'C','carriage')

# Gear pairs are recorded explicitly to audit mechanical direction.
pairs=[];shaft_groups=[]
def gear(name,part,x,y,z,ac='fixed'):
 return native(name,part,[x,y,z],actor=ac)
def axle(name,part,x,y,z,ac='fixed'):
 return native(name,part,[x,y,z],np.eye(3),ac)
def pair(a,b):pairs.append([a,b])
def train(names):pairs.extend([list(p) for p in zip(names,names[1:])])
# Carry !Bprime directly: Sub=0 uses one mesh, Sub=1 uses two.
height=10.2+np.sqrt(80)
gear('X B direct','4019.dat',16,10.2,16,'X')
gear('X B reverse drive','4019.dat',-32.4,10.2,16,'X')
for x,suffix in [(-32.4,'back'),(-16,'front')]:gear('X compound '+suffix,'10928.dat',x,height,24,'X')
pair('X B direct','X L072');pair('X B reverse drive','X compound back');pair('X compound front','X L102')
shaft_groups.append(['X compound back','X compound front'])
axle('X compound idler axle','3705.dat',-24.2,height,24,'X')
for xx in [-10,-38.4]:native('X compound retainer '+str(xx),'32123a.dat',[xx,height,24])
# !Bprime is one row below P: direct gives Bprime; compound gives !Bprime.
x=mods['P'][0]
gear('P direct input','4019.dat',x+16,10.2,32,'P');pair('P direct input','P L072')
gear('P reversing input','4019.dat',x,10.2,32,'P')
for xx,suffix in [(x,'back'),(x-16,'front')]:gear('P compound '+suffix,'10928.dat',xx,height,40,'P')
pair('P reversing input','P compound back');pair('P compound front','P L102')
shaft_groups.append(['P compound back','P compound front'])
axle('P compound idler axle','3705.dat',x-8,height,40,'P')
for xx in [x+6,x-22]:native('P compound retainer '+str(xx),'32123a.dat',[xx,height,40])
# Carry direct input uses the same !Bprime rail, on the other side of the clutch.
x=mods['C'][0]
gear('C inverted Bprime input','4019.dat',x+16,10.2,32,'C');pair('C inverted Bprime input','C L072')
# A stays at Z=0. Offset compound idlers provide the positive branches.
for ac,side in [('C',-1),('S',1)]:
 x=mods[ac][0]
 back=x+side*32.4 if ac=='C' else x
 gear(ac+' A reversing drive','4019.dat',back,10.2,0,ac)
 for xx,suffix in [(back,'back'),(x+side*16,'front')]:gear(ac+' A compound '+suffix,'10928.dat',xx,height,8,ac)
 pair(ac+' A reversing drive',ac+' A compound back');pair(ac+' A compound front',ac+(' L102' if side<0 else ' L072'))
 shaft_groups.append([ac+' A compound back',ac+' A compound front'])
 axle(ac+' compound idler axle','3705.dat',(x+side*24.2 if ac=='C' else x+8),height,8,ac)
 for xx in ([x+side*10,x+side*38.4] if ac=='C' else [x-6,x+22]):native(ac+' compound retainer '+str(xx),'32123a.dat',[xx,height,8])
x=mods['S'][0]
gear('S A direct','4019.dat',x-16,10.2,0,'S');pair('S A direct','S L102')
# Axles are standard stock, cut-free; connectors sit between rotating gear planes.
axle('B input','23948.dat',0,10.2,16)
axle('Sub input','3707.dat',0,10.2,64)
axle('Cin input','3707.dat',mods['P'][0],10.2,80)
# Continuous !Bprime rail, independent of the offset P output.
axle('Not Bprime rail first','3707.dat',0,10.2,32)
axle('Not Bprime rail middle','3737.dat',72,10.2,32)
axle('Not Bprime rail last','60485.dat',mods['C'][0],10.2,32)
for xx in [32,mods['P'][0]+36]:native('Not Bprime connector '+str(xx),'59443.dat',[xx,10.2,32])
axle('P rail first','60485.dat',mods['P'][0],10.2,48)
axle('P rail continuation','60485.dat',mods['C'][0],10.2,48)
native('P rail connector','59443.dat',[mods['P'][0]+36,10.2,48])
axle('A rail first','3737.dat',mods['C'][0]-8,10.2,0)
axle('A rail continuation','60485.dat',mods['C'][0]+68,10.2,0)
native('A rail connector','59443.dat',[mods['C'][0]+32,10.2,0])
for ac in ['C','S']:axle(ac+' output axle','3707.dat',mods[ac][0],10.2,16,ac)
# External stops retain the output axles; their clutch gears run between plate faces.
for ac,z,sides in [('X',32,[-30]),('P',48,[-30]),('C',16,[-30,30]),('S',16,[-30,30])]:
 for off in sides:native(ac+' output outer retainer '+str(off),'32123a.dat',[mods[ac][0]+off,10.2,z],actor=ac)
for off in [-30,30]:native('A end rail retainer '+str(off),'32123a.dat',[mods['S'][0]+off,10.2,0])
native('Not Bprime end retainer','32123a.dat',[mods['C'][0]+30,10.2,32])
# Two-pin bearing plates, including the off-axis compound idler bearings.
configs={
 'X':[(-24,[(10.2,16),(height,24),(10.2,32)]),(28,[(10.2,16)])],
 'P':[(-24,[(10.2,32),(10.2,48)]),(24,[(10.2,32),(10.2,48)])],
 'C':[(-24,[(10.2,0),(height,8),(10.2,16),(10.2,32)]),(24,[(10.2,16),(10.2,32)])],
 'S':[(-24,[(10.2,0),(10.2,16)]),(24,[(10.2,0),(10.2,16)])]
}
for ac,entries in configs.items():
 for xx,holes in entries:
  hz=[z for y,z in holes];name=ac+' signal bearing '+str(xx)
  sh=wall(name,mods[ac][0]+xx,holes,[min(hz)-9,max(hz)+9],([min(hz)-4,max(hz)+4] if len(hz)==1 else [min(hz),max(hz)]))
  if (ac=='P' and xx==-24) or (ac=='S' and xx==24):
   sh-=cyl([mods[ac][0]+xx,height,40 if ac=='P' else 8],4.1,10,'x')
  if ac=='C':
   guide=ac+' guide bearing '+str(xx)
   sh+=solids.pop(guide)
   prints[:]=[p for p in prints if p['id']!=guide]
  put(name,sh,FX,ac)
for ac,xx,z in [('P',-8,40),('S',8,8)]:
 name=ac+' compound bearing';sh=wall(name,mods[ac][0]+xx,[(height,z)],[z-12,z+12],[z-8,z+8]);put(name,sh,FX,ac)
xmin=min(x for x,z in mounts)-6;xmax=max(x for x,z in mounts)+6
zmin=min(z for x,z in mounts)-6;zmax=max(z for x,z in mounts)+6
base=box([xmin,28,zmin],[xmax,36.2,zmax])
for x,z in mounts:base=pin_cut(base,[x,28,z],length=20)
put('Base',base,FY)
D=dict(name='Shared-actuator shaft-aligned adder',records=records,prints=prints,actors={k:v.tolist() for k,v in actors.items()},orientations={k:1 for k in mods},mounts=mounts,gear_pairs=pairs,shaft_groups=shaft_groups,base_mm=[xmax-xmin,zmax-zmin,8.2],status='Engineering prototype — sampled geometry checks pass; loaded switching and print validation pending',ports={'A':[mods['C'][0]-40,10.2,0],'B':[44,10.2,16],'SU':[-34,10.2,64],'Cin':[pitch+30,10.2,80],'Sum':[mods['S'][0]+30,10.2,16],'Cout':[mods['C'][0]+30,10.2,16]})
(OUT/'Assembly manifest.json').write_text(json.dumps(D,indent=2))
print('Built',len(prints),'parts; base',D['base_mm'],flush=True)

# Remove transient bearings merged into the final compound supports.
for name in ["C guide bearing -24.stl", "C guide bearing 24.stl"]:
 (OUT/name).unlink(missing_ok=True)
