import os
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT;D=json.loads((OUT/'Assembly manifest.json').read_text());P=OUT/'Print parts';P.mkdir(exist_ok=True);parts=[];checks=[]
for p in D['prints']:
 t=trimesh.load(OUT/p['path']);R=np.asarray(p['rotation']);tt=t.copy();tt.vertices=tt.vertices@R.T;shift=-tt.bounds[0];tt.vertices+=shift
 assert tt.is_watertight and tt.is_winding_consistent,p['id'];assert len(tt.split())==1,p['id']
 area=float(tt.area_faces[np.all(np.abs(tt.triangles[:,:,2])<1e-4,axis=1)].sum());assert area>5,(p['id'],area)
 n=p['id'];entries=[]
 if p['bores']:
  for c in p['bores']:c=np.array(c);c[0]=t.bounds[0,0];entries.append(c)
 elif p['actor'] in D['actors']:
  name=n[2:];orig=trimesh.load(ROOT/'Sources/Inputs/multiplexer'/(name+'.stl'));lo=orig.bounds[0];local=[]
  if 'carriage half' in n or 'Bearing wall' in n:local=[[lo[0],10.2,z] for z in [24,32,40]]
  elif 'actuator bridge' in n or 'Rear bridge' in n:local=[[x,y,54.8 if 'Rear bridge' in n else 19.8] for x,y in [(0,2.2),(14.500924592586399,-3.261892187851191)]]
  elif 'lever' in n:local=[[14.500924592586399,-3.261892187851191,24]]
  sg=D['orientations'][p['actor']];d=np.array(D['actors'][p['actor']]);entries=[np.array(c)*[sg,1,sg]+d for c in local]
 heights=[float((c@R.T+shift)[2]) for c in entries];assert all(abs(z)<.001 for z in heights),(n,heights)
 tt.export(P/(n+'.stl'));checks.append(dict(part=n,watertight=True,shells=1,bed_contact_area_mm2=area,bearing_entrance_heights_mm=heights,rotation=R.tolist(),translation=shift.tolist(),size_mm=tt.extents.tolist()))
 parts.append(dict(name=n,mesh=tt,w=float(tt.extents[0]),h=float(tt.extents[1])))
# Reversible fit coupon: bearing bores start on bed. Counted separately from installed parts.
s=m.Manifold.cube([48,14,7])
for i,diam in enumerate([5.2,5.3,5.4]):
 s=s-m.Manifold.cylinder(9,diam/2,circular_segments=80).translate([8+16*i,7,-1])
 for j in range(i+1):s=s-m.Manifold.cube([1,2,2]).translate([5+16*i+2*j,0,5])
a=s.to_mesh64();coupon=trimesh.Trimesh(np.asarray(a.vert_properties)[:,:3],np.asarray(a.tri_verts),process=True);coupon.export(P/'Bearing fit coupon.stl');parts.append(dict(name='Bearing fit coupon',mesh=coupon,w=48,h=14))
# Base has its own plate. Remaining parts packed with 6 mm rectangular spacing.
base=next(x for x in parts if x['name']=='Base');bins=[[dict(item=base,x=0,y=0,w=base['w'],h=base['h'],rot=False)]]
for item in sorted([x for x in parts if x is not base],key=lambda x:x['w']*x['h'],reverse=True):
 best=None
 for bi in range(1,len(bins)+1):
  placed=bins[bi] if bi<len(bins) else []
  xs={0}|{p['x']+p['w']+6 for p in placed};ys={0}|{p['y']+p['h']+6 for p in placed}
  for rotated in [False,True]:
   w,h=(item['h'],item['w']) if rotated else (item['w'],item['h'])
   for x in xs:
    for y in ys:
     if x+w>246.001 or y+h>246.001:continue
     if any(x<p['x']+p['w']+5.999 and x+w+5.999>p['x'] and y<p['y']+p['h']+5.999 and y+h+5.999>p['y'] for p in placed):continue
     score=(bi,max([y+h]+[p['y']+p['h'] for p in placed]),x+w,x)
     if best is None or score<best[0]:best=(score,bi,x,y,w,h,rotated)
 if best is None:raise RuntimeError('Cannot pack '+item['name'])
 _,bi,x,y,w,h,rr=best
 if bi==len(bins):bins.append([])
 bins[bi].append(dict(item=item,x=x,y=y,w=w,h=h,rot=rr))
plates=[]
for bi,placed in enumerate(bins):
 arr=[];names=[]
 for p in placed:
  t=p['item']['mesh'].copy()
  if p['rot']:t.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,0,1]));t.vertices-=t.bounds[0]
  t.vertices+=[p['x'],p['y'],0];arr.append(t);names.append(p['item']['name'])
 plate=trimesh.util.concatenate(arr);name=f'Print plate {bi+1}.stl';plate.export(OUT/name);plates.append(dict(file=name,parts=names,size_mm=plate.extents.tolist(),minimum_rectangular_gap_mm=6 if len(arr)>1 else None));assert len(plate.split())==len(arr)
(OUT/'Print checks.json').write_text(json.dumps(dict(parts=checks,plates=plates,coupon='Separate fit coupon; three bores marked with 1/2/3 notches = 5.2/5.3/5.4 mm'),indent=2)+'\n')
(OUT/'Print file hashes.json').write_text(json.dumps({str(p.relative_to(OUT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(OUT.rglob('*.stl'))},indent=2)+'\n');print(json.dumps(plates,indent=2),flush=True)
