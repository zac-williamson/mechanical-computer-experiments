from pathlib import Path
import json,numpy as np,trimesh,shutil
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';D=Path(__file__).resolve().parents[1]/'Planar register';rows=json.load(open(O/'Print orientations.json'))['parts'];gap=5.;free=[(0.,0.,246.,246.)];placements=[];meshes=[]
for row in sorted(rows,key=lambda p:max(p['size_mm'][:2]),reverse=True):
 t=trimesh.load(O/'Prototype print parts'/(row['part']+'.stl'));assert t.is_watertight and row['flat_bed_area_mm2']>1
 w,h=t.extents[:2];options=[]
 for i,(x,y,fw,fh) in enumerate(free):
  for turn,(a,b) in enumerate([(w+gap,h+gap),(h+gap,w+gap)]):
   if a<=fw and b<=fh:options.append(((min(fw-a,fh-b),max(fw-a,fh-b)),i,turn,a,b))
 assert options,('Does not fit',row['part'])
 _,i,turn,a,b=min(options);x,y,_,_=free[i];used=(x,y,a,b);nf=[]
 for fx,fy,fw,fh in free:
  if x>=fx+fw or x+a<=fx or y>=fy+fh or y+b<=fy:nf.append((fx,fy,fw,fh));continue
  if x>fx:nf.append((fx,fy,x-fx,fh))
  if x+a<fx+fw:nf.append((x+a,fy,fx+fw-x-a,fh))
  if y>fy:nf.append((fx,fy,fw,y-fy))
  if y+b<fy+fh:nf.append((fx,y+b,fw,fy+fh-y-b))
 free=[r for i,r in enumerate(nf) if not any(i!=j and r[0]>=s[0] and r[1]>=s[1] and r[0]+r[2]<=s[0]+s[2] and r[1]+r[3]<=s[1]+s[3] and (r!=s or j<i) for j,s in enumerate(nf))]
 if turn:t.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,0,1]))
 t.apply_translation(-t.bounds[0]);t.apply_translation([x+5,y+5,0]);meshes.append(t);placements.append(dict(part=row['part'],bounds_mm=t.bounds.tolist(),turn_deg=turn*90))
for i,a in enumerate(meshes):
 for b in meshes[i+1:]:
  sep=np.maximum(a.bounds[0,:2]-b.bounds[1,:2],b.bounds[0,:2]-a.bounds[1,:2]);assert sep.max()>=gap-1e-4
out=trimesh.util.concatenate(meshes);path=D/'Print layout.stl';out.export(path,file_type="stl_ascii");check=trimesh.load(path);assert check.is_watertight and len(check.split())==len(rows);assert abs(check.bounds[0,2])<1e-5
r=dict(parts=len(rows),size_mm=out.extents.tolist(),minimum_part_spacing_mm=gap,units='mm',placements=placements);(D/'Print layout.json').write_text(json.dumps(r,indent=2));shutil.copy2(O/'Print orientations.json',D/'Print orientations.json');shutil.copytree(O/'Prototype print parts',D/'Prototype print parts',dirs_exist_ok=True);print(path);print('Parts',len(rows),'size',out.extents)
