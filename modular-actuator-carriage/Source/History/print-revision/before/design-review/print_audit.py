from pathlib import Path
import json,numpy as np,trimesh
R=Path(__file__).resolve().parents[1];O=R/'adapted'
D=json.loads((O/'Model.json').read_text());normals={'left':[-1,0,0],'right':[1,0,0],'front':[0,-1,0],'rear':[0,1,0],'bottom':[0,0,-1]};out=[]
for p in D['parts']:
 if p['kind']!='printed':continue
 t=trimesh.load(O/(p['name']+'.stl'));original=t.triangles_center.copy();t.apply_transform(trimesh.geometry.align_vectors(normals[p['bed']],[0,0,-1]));t.apply_translation(-t.bounds[0])
 c=t.triangles_center;n=t.face_normals;area=t.area_faces
 ids=np.where((n[:,2]<-.7072)&(c[:,2]>.15)&(area>.05))[0]
 groups={}
 for i in ids:
  key=(round(float(c[i,2]),1),tuple(np.round(n[i],2)))
  g=groups.setdefault(key,dict(area_mm2=0,print_height=key[0],print_normal=key[1],largest_face_area=0))
  g['area_mm2']+=float(area[i])
  if area[i]>g['largest_face_area']:g.update(largest_face_area=float(area[i]),model_point=original[i].tolist(),print_point=c[i].tolist())
 rows=sorted(groups.values(),key=lambda x:x['area_mm2'],reverse=True)
 # Downward ray from largest flat underside patches finds vertical clearance below them.
 for g in rows[:12]:
  orig=np.array(g['print_point']);loc,_,_=t.ray.intersects_location([orig-[0,0,.0002]],[[0,0,-1]],multiple_hits=False)
  g['vertical_gap_below_mm']=float(orig[2]-loc[0,2]) if len(loc) else float(orig[2])
 out.append(dict(part=p['name'],bed_face=p['bed'],print_height=float(t.extents[2]),bed_contact_area_mm2=float(area[(n[:,2]<-.999)&(c[:,2]<.01)].sum()),steep_underside_area_mm2=float(area[ids].sum()),largest_underside_patches=rows[:12]))
print(json.dumps(out,indent=2),flush=True);(R/'design-review/Print geometry audit.json').write_text(json.dumps(out,indent=2))
