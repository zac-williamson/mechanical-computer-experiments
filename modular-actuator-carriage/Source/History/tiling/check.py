from pathlib import Path
import os,runpy,json,base64,gzip
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'tile-candidate';os.environ['PLANAR_OUTPUT']=str(O)
runpy.run_path(str(R/'validate.py'),run_name='__main__')
h=(O/'Viewer.html').read_text();D=json.loads(h.split('<script type="application/json" id="data">')[1].split('</script>')[0]);T=D['tiling']
def unpack(s):return np.frombuffer(gzip.decompress(base64.b64decode(s)),dtype='<f4').reshape(-1,3)
v=unpack(D['geometry']);tv=unpack(T['geometry'])
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=40).rotate([-90,0,0]).translate([c[0],a,c[2]])
def get(p,v):
 a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 if p['kind']=='native' and 'pin' in p['name'].lower():
  # Running-space proxy excludes nominal pin compression in its intended bore.
  bb=t.bounds;c=bb.mean(0)
  if p['name'].startswith('Rod coupling pin'):
   s=cy(2.4,3.8,12.6,c)+cy(3.2,5.4,6.2,c)+cy(2.6,12.6,13.4,c)
  else:s=cy(2.45,bb[0,1],bb[1,1],c)+cy(3.2,c[1]-.4,c[1]+.4,c)
 if s.status()!=m.Error.NoError:
  # Conservative convex envelope only for an open native reference mesh.
  t=t.convex_hull;s=m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
 return s
parts=D['parts'];base=[get(p,v) for p in parts];poses=[];bounds=[]
for k in range(len(D['frames'])):
 ss=[]
 for p,s in zip(parts,base):
  if p['name'] in D['bands'][k]:s=get(dict(D['bands'][k][p['name']],kind='elastic'),v)
  else:s=s.transform(np.array(D['transforms'][k][p['name']]).reshape(4,4).T[:3])
  ss.append(s)
 poses.append(ss);bounds.append(np.array([s.bounding_box() for s in ss]).reshape(-1,2,3))
hits=[];tests=0;cache={}
def overlap(a,b,ba,bb,key,desc):
 global tests
 if np.any(ba[1]<=bb[0]+1e-5) or np.any(bb[1]<=ba[0]+1e-5):return
 if key in cache:return
 tests+=1;q=a^b;vol=q.volume();cache[key]=vol
 if vol>.02:hits.append(dict(**desc,volume_mm3=vol,bounds=q.bounding_box()))
for shift in [(80,0,0),(0,0,64),(80,0,64),(-80,0,64)]:
 sh=np.array(shift)
 for i in range(len(poses)):
  js=[i] if shift==(80,0,0) else range(len(poses))
  for j in js:
   ba=bounds[i];bb=bounds[j]+sh
   mask=np.all(ba[:,None,1,:]>bb[None,:,0,:]+1e-5,axis=2)&np.all(bb[None,:,1,:]>ba[:,None,0,:]+1e-5,axis=2)
   for a,b in np.argwhere(mask):
    ma=parts[a]['motion'];mb=parts[b]['motion'];key=('modules',shift,int(a),int(b),i if ma!='fixed' else 0,j if mb!='fixed' else 0)
    overlap(poses[i][a],poses[j][b].translate(sh),ba[a],bb[b],key,dict(type='between modules',shift=shift,poses=[i,j],a=parts[a]['name'],b=parts[b]['name']))
 print('Neighbour displacement checked',shift,flush=True)
extras=[get(p,tv) for p in T['parts']]
# Added hardware against every module; horizontal neighbours share the same pose.
for ei,(ep,es) in enumerate(zip(T['parts'],extras)):
 for k,f in enumerate(D['frames']):
  esh=[f['q'] if ep['motion']=='carriage' else f['lock'] if ep['motion']=='lock' else 0,0,0];se=es.translate(esh);be=np.array(se.bounding_box()).reshape(2,3)
  for row in [0,1]:
   for col in [0,1]:
    sh=np.array([80*col,0,64*row])
    # Different rows cannot touch these rod pins (disjoint Z bounds), but
    # exercise all pose combinations if they ever acquire matching bounds.
    js=[k] if ep['motion']=='fixed' or row==ep['row'] else range(len(poses))
    for j in js:
     for pi,(p,s) in enumerate(zip(parts,poses[j])):
      bb=bounds[j][pi]+sh
      if np.any(be[1]<=bb[0]+1e-5) or np.any(bb[1]<=be[0]+1e-5):continue
      key=('extras',ei,k if ep['motion']!='fixed' else 0,row,col,pi,j if p['motion']!='fixed' else 0)
      overlap(se,s.translate(sh),be,bb,key,dict(type='connector to module',a=ep['name'],b=p['name'],module=[col,row],poses=[k,j]))
# Extras against each other, including bridge pins and translating coupling pins.
for k,f in enumerate(D['frames']):
 ss=[s.translate([f['q'] if p['motion']=='carriage' else f['lock'] if p['motion']=='lock' else 0,0,0]) for p,s in zip(T['parts'],extras)]
 for i,s in enumerate(ss):
  for j in range(i):
   overlap(s,ss[j],np.array(s.bounding_box()).reshape(2,3),np.array(ss[j].bounding_box()).reshape(2,3),('extra pairs',i,j,k),dict(type='connector pair',a=T['parts'][i]['name'],b=T['parts'][j]['name'],pose=k))
quality=[]
for name in ['Module base','Horizontal frame bridge','Vertical frame bridge','Centre frame bridge']:
 t=trimesh.load(O/(name+'.stl'));quality.append(dict(part=name,watertight=bool(t.is_watertight),solids=len(t.split()),bounds=t.bounds.tolist()))
report=dict(passed=not hits and all(p['watertight'] and p['solids']==1 for p in quality),pitch_X_mm=80,pitch_Z_mm=64,poses_per_row=len(poses),vertical_pose_combinations=len(poses)**2,boolean_tests=tests,hits=hits,quality=quality,scope='Same orientation, 2x2. Corresponding rods coupled along X; rows can move independently. Nominal pin compression excluded by 2.45 mm shaft-radius proxies; no strength or drive-torque claim.')
(O/'Tiling checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
# Bridges press onto preinstalled frame pins; rod half-pins push in afterwards.
assembly_hits=[];assembly_tests=0
for ei,(ep,es) in enumerate(zip(T['parts'],extras)):
 if ep['kind']!='printed' and not ep['name'].startswith('Rod coupling pin'):continue
 for k in [0,13,26]:
  f=D['frames'][k];qx=f['q'] if ep['motion']=='carriage' else f['lock'] if ep['motion']=='lock' else 0
  for travel in np.linspace(-8,0,9):
   se=es
   if ep['name'].startswith('Rod coupling pin') and travel<0:
    # The 0.1 mm snap lip compresses through the bore; check its shaft path.
    b=np.array(es.bounding_box()).reshape(2,3);c=b.mean(0);se=cy(2.4,3.8,13.4,c)+cy(3.2,5.4,6.2,c)
   se=se.translate([qx,float(travel),0]);be=np.array(se.bounding_box()).reshape(2,3)
   for row in [0,1]:
    for col in [0,1]:
     shift=np.array([80*col,0,64*row])
     for pi,p in enumerate(parts):
      bb=bounds[k][pi]+shift
      if np.any(be[1]<=bb[0]+1e-5) or np.any(bb[1]<=be[0]+1e-5):continue
      assembly_tests+=1;q=se^poses[k][pi].translate(shift)
      if q.volume()>.02:assembly_hits.append(dict(part=ep['name'],other=p['name'],travel=float(travel),pose=k,module=[col,row],volume=q.volume()))
report['assembly_access']=dict(passed=not assembly_hits,boolean_tests=assembly_tests,hits=assembly_hits,sequence='Preinstall 2L frame pins, press bridges +Y, then insert 4274 rod pins +Y. Rod-pin lip compression through intended holes is excluded during insertion.')
report['passed']=report['passed'] and not assembly_hits
(O/'Tiling checks.json').write_text(json.dumps(report,indent=2));print('Assembly access',report['assembly_access'],flush=True)
