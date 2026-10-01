import os
"""Create a review viewer with explicit poses; no print-release claims."""
from pathlib import Path
import json,base64,gzip,math
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parent;O=Path(os.environ.get('PLANAR_OUTPUT',str(R/'adapted')));ROOT=R.parents[1]
D=json.loads((O/'Model.json').read_text());v=np.frombuffer(gzip.decompress(base64.b64decode(D['geometry'])),dtype='<f4').reshape(-1,3)
arrays=[v];offset=v.size
trace=json.loads((ROOT/'latest-register-analysis/planar-register/work/register-mux-reference/multiplexer/Switching trace.json').read_text())['frames']
def cy(r,a,b,c):return m.Manifold.cylinder(b-a,r,circular_segments=32).translate([*c,a])
def loop(a,b,inner,outer,z0,z1):
 def cap(r):return (cy(r,z0,z1,a)+cy(r,z0,z1,b)).hull()
 return cap(outer)-cap(inner)
def mesh(s):
 a=s.to_mesh64();return trimesh.Trimesh(a.vert_properties[:,:3],a.tri_verts,process=False)
def appendmesh(t,name):
 global offset
 a=t.triangles.reshape(-1,3);p=dict(name=name,offset=offset,vertices=len(a));arrays.append(a);offset+=a.size;return p
lo=0;hi=8.25
for _ in range(60):
 cx=(lo+hi)/2
 if math.sqrt(144-(3.75-cx)**2)-math.sqrt(144-(-3.75-cx)**2)<3.8:lo=cx
 else:hi=cx
cx=(lo+hi)/2;cz=47.6+math.sqrt(144-(3.75-cx)**2)
D['labels']={'U015':'Actuator worm','U022':'Actuator reaction gear','L072':'Left clutch gear','L102':'Right clutch gear','L099':'Sliding clutch ring','L097':'Clutch axle connector','L069':'Right output axle bush','L105':'Left output axle bush','C-shaft':'Actuator input axle','reaction-stop-axle':'Reaction gear axle','reaction-retainer':'Reaction gear axle bush','pivot-stop-axle':'Lever pivot axle','pivot-retainer':'Lever pivot bush'}
D['frames']=[];D['transforms']=[];D['bands']=[]
seq=[(-3.75,float(l),'Release left position') for l in np.linspace(0,3.8,5)]+[(float(q),3.8,'Move carriage right') for q in np.linspace(-3.75,3.75,17)]+[(3.75,float(l),'Lock right position') for l in np.linspace(3.8,0,5)]
for q,lift,label in seq:
 f=min(trace,key=lambda f:abs(f['q']-q));beta=f['b'];lock=math.sqrt(144-(cz-47.6-lift)**2)-cx
 D['frames'].append(dict(q=q,lift=lift,lock=lock,label=label));trans={};bands={}
 rot=trimesh.transformations.rotation_matrix(math.radians(-beta),[0,0,1],[13.192323604,26.328448698,16])
 for p in D['parts']:
  mo=p['motion'];M=rot.copy() if mo=='rocker' else np.eye(4)
  if mo in ['carriage','worm','clutch-ring']:M[0,3]=q
  if mo=='bolt':M[2,3]=lift
  if mo=='lock':M[0,3]=lock
  trans[p['name']]=M.T.reshape(-1).tolist()
  if p['kind']=='elastic':
   if mo=='lock-band':
    x=-13.1 if p['name'].startswith('Left') else 3
    s=loop([27.6,42.8],[27.6,49.6+lift],1.5,2.1,x-.5,x+.5).rotate([90,0,90])
   else:
    anchor=trimesh.transform_points([[22.192323604,24.828448698,16]],rot)[0]
    s=loop([36.592323604,22.328448698],anchor[:2],2,3.2,15.4,16.6)
   bands[p['name']]=appendmesh(mesh(s),p['name'])
 D['transforms'].append(trans);D['bands'].append(bands)
D['geometry']=base64.b64encode(gzip.compress(np.concatenate(arrays).astype('<f4').tobytes())).decode()
# Bed orientations are provided for inspection, not a validated print release.
normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]}
pa=[];pm=[];x=y=row=0;off=0;checks=[]
for p in D['parts']:
 if p['kind']!='printed':continue
 t=trimesh.load(O/(p['name']+'.stl'));t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed']],[0,0,-1]));t.apply_translation(-t.bounds[0]);w,h,z=t.extents
 if x+w>230:x=0;y+=row+8;row=0
 t.apply_translation([x,y,0]);x+=w+8;row=max(row,h)
 a=t.triangles.reshape(-1,3);pa.append(a);pm.append(dict(name=p['name'],offset=off,vertices=len(a),color=p['color']));off+=a.size
 checks.append(dict(part=p['name'],watertight=bool(t.is_watertight),bounds=t.bounds.tolist(),bed_face=p['bed']))
D['print_parts']=pm;D['print_geometry']=base64.b64encode(gzip.compress(np.concatenate(pa).astype('<f4').tobytes())).decode();D['print_centre']=np.concatenate(pa).reshape(-1,3).max(0).tolist();D['print_centre']=[a/2 for a in D['print_centre']]
(O/'Bed orientation checks.json').write_text(json.dumps(checks,indent=2))
if (O/'Tiling geometry.json').exists():D['tiling']=json.loads((O/'Tiling geometry.json').read_text())
html=(R/'final_viewer.html').read_text().replace('__DATA__',json.dumps(D,separators=(',',':')))
html=html.replace('Print layout</button>','Bed orientations</button>').replace('<a href="Print%20layout.stl">Print layout STL</a> · <a href="Prototype%20package.zip">All parts and instructions</a> · ','').replace('CAD prototype. Motion is an inspection sequence; physical fits, band tension and loaded operation still need a print test.','WORK IN PROGRESS — not a print release. The carriage split places the clutch-contact face on the bed. Test the bearing-shoulder bridges on the supplied small sample before printing a full carriage. The sequence shows sampled positions, not a verified driven cycle. Full-module assembly, band tension and loaded operation remain unverified.')
(O/'Viewer.html').write_text(html)
(O/'Source provenance.json').write_text(json.dumps(json.loads((O/'Development checks.json').read_text())['provenance'],indent=2))
print('Updated review viewer; no print layout STL released.',flush=True)
