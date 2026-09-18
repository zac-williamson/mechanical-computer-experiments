from pathlib import Path
import json,shutil,collections,os
import numpy as np,trimesh,manifold3d as m
ROOT=Path(__file__).resolve().parent;OUT=Path(os.environ.get('REGISTER_OUTPUT',str(ROOT.parent)));OUT.mkdir(exist_ok=True)
SRC=ROOT.parents[1]/'register';D=json.loads((SRC/'Assembly manifest.json').read_text())
keep={'U015','U022','U185','reaction-axle','reaction-retainer-14','reaction-retainer-50','reaction-spacer-24','reaction-spacer-40','pivot-axle-with-stop','U017','U019','U032','SECOND-GUIDE','SECOND-GUIDE-BUSH-R','SECOND-GUIDE-BUSH-L','pivot-front-half-bush','pivot-hub-rear-bush','pivot-rear-retainer','reaction-rear-extra-half-bush','selector-right-retainer','Carriage joining pin top','Carriage joining pin base','C-shaft'}
prints=[p for p in D['prints'] if p['actor'] in ['K','W','lock','link'] and not p['id'].startswith(('K inner','K outer'))]
records=[]
for r in D['records']:
 n=r['record_id']
 if n.startswith('W ') or r['actor'] in ['link','lock'] or n=='Shaft connector -40' or (n.startswith('K ') and (n[2:] in keep or n.startswith('K frame pin '))):records.append(r)
for p in prints:shutil.copy(SRC/p['path'],OUT/p['path'])
mounts=[(r['pos'][0]*.4,r['pos'][2]*.4) for r in records if r['part']=='2780.dat' and abs(r['pos'][1]*.4-28)<1e-5]
assert len(set(mounts))==len(mounts)
# Thin through-hole base. All retained support pin centres have at least 6 mm edge distance.
lo=np.floor(np.min(mounts,axis=0)-6);hi=np.ceil(np.max(mounts,axis=0)+6)
for p in prints:
 if p['motion']=='fixed':
  bb=trimesh.load(OUT/p['path']).bounds[:,[0,2]];lo=np.minimum(lo,np.floor(bb[0]-2));hi=np.maximum(hi,np.ceil(bb[1]+2))
base=m.Manifold.cube([hi[0]-lo[0],8.2,hi[1]-lo[1]]).translate([lo[0],28,lo[1]])
for x,z in mounts:
 for radius,height in [(2.46,20),(3.3,1.2)]:base=base-m.Manifold.cylinder(height,radius,circular_segments=64,center=True).rotate([90,0,0]).translate([x,28,z])
a=base.to_mesh64();mesh=trimesh.Trimesh(np.asarray(a.vert_properties)[:,:3],np.asarray(a.tri_verts),process=True);assert mesh.is_watertight;mesh.export(OUT/'Base.stl')
prints.append(dict(id='Base',actor='fixed',motion='fixed',rotation=[[1,0,0],[0,0,1],[0,-1,0]],bores=[],sliding=[],path='Base.stl'))
d=dict(name='Two-actuator cam test rig',status='Physical prototype; friction and loaded timing require testing',records=records,prints=prints,actors={k:D['actors'][k] for k in ['K','W']},mounts=mounts,hold=D['hold'],stroke=D['stroke'],base_mm=[float(hi[0]-lo[0]),float(hi[1]-lo[1]),8.2],ports={'CAM CONTROL':D['ports']['W'],'TEST DRIVE':D['ports']['D']},operation='W retracts lock before its clutch drives K from TEST DRIVE. K has no output clutch or power gear train.')
(OUT/'Assembly manifest.json').write_text(json.dumps(d,indent=2));(OUT/'LEGO parts.csv').write_text('Part,Quantity\n'+''.join(f'{p},{n}\n' for p,n in sorted(collections.Counter(r['part'] for r in records).items())))
print(len(prints),'printed parts;',len(records),'LEGO components;',len(mounts),'base pins; base',d['base_mm'])
