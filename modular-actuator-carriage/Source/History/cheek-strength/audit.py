from pathlib import Path
import os,json,runpy
import numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'strength-candidate';A=R/'simplified-carriage-r1';os.environ['PLANAR_OUTPUT']=str(O)
def solid(t):return m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64)))
D=json.loads((O/'Model.json').read_text());normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]};rows=[]
for p in D['parts']:
 if p['kind']!='printed':continue
 t=trimesh.load(O/(p['name']+'.stl'));row={'part':p['name'],'watertight':bool(t.is_watertight),'solids':len(t.split()),'neck_flags':[],'detached_islands':[]};assert row['watertight'] and row['solids']==1
 t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed']],[0,0,-1]));t.apply_translation(-t.bounds[0]);s=solid(t)
 for h in np.arange(.3,t.extents[2],.2):
  sec=s.slice(h);prev=s.slice(h-.2)
  if any(c.area()>.1 and (c^prev.offset(.21)).area()<.01 for c in sec.decompose()):row['detached_islands'].append(round(float(h),2))
 for h in np.arange(.5,t.extents[2],1):
  for c in s.slice(h).decompose():
   pieces=[q for q in c.offset(-1).decompose() if q.area()>4]
   if len(pieces)>1:row['neck_flags'].append({'height':float(h),'pieces':len(pieces),'bounds':c.bounds()})
 rows.append(row);print(p['name'],'islands',row['detached_islands'],'neck layers',len(row['neck_flags']),flush=True)
(O/'All-part print screen.json').write_text(json.dumps(rows,indent=2))
# Explicit comparison of the failed mounting neck at its narrow section.
neck=[]
for name,z in [('Upper actuator cheek',20.4),('Lower actuator cheek',7.6)]:
 for label,folder in [('before',A),('after',O)]:
  s=solid(trimesh.load(folder/(name+'.stl')));p=m.Manifold.cube([15,.02,3.98]).translate([25,26.49,z+.01]);neck.append({'part':name,'version':label,'neck_effective_width_X_mm':(s^p).volume()/(.02*3.98)})
(O/'Neck measurements.json').write_text(json.dumps(neck,indent=2));print(neck,flush=True)
