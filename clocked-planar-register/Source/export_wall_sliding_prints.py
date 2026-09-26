"""Export rod/guide orientations and audit their actual sliding faces for support contact.

Checks the four planar sides of every rod/guide interface over the full nominal
stroke. Only bed faces, vertical walls and upward faces are accepted as running
faces; downward overhangs above the bed fail. Static non-running overhangs are
reported separately. The entire detached CLOCK pickup is screened as well.
"""
from pathlib import Path
import json,hashlib,numpy as np,trimesh
O=Path(__file__).resolve().parents[1]/'Wall register';D=O/'Print oriented sliding parts';D.mkdir(exist_ok=True)
for f in D.glob('*.stl'):f.unlink()
P={p['id']:p for p in json.loads((O/'parts.json').read_text())};V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3)
G=json.loads((O/'Rod guide assembly schedule.json').read_text())['connections']
rod_names={('bit',k):'bit '+k+' vertical control rod' for k in ['clock','write']}
rod_names.update({('control',k):'Control '+k.upper()+' direct rod and pickup' for k in ['clock','write']})
pickup='Control CLOCK detachable pickup'
orient={n:1 for n in rod_names.values()};orient[pickup]=-1
for e in G:orient[e['part']]=1;orient[e['host']]=-1
meshes={};exports=[]
for n,sign in orient.items():
 p=P[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);meshes[n]=t
 rot=trimesh.geometry.align_vectors([0,sign,0],[0,0,1]);printed=t.copy();printed.apply_transform(rot);shift=-printed.bounds[0];printed.apply_translation(shift)
 bad=(printed.face_normals[:,2]<-.70712)&(printed.triangles_center[:,2]>.001)
 path=D/(n+'.stl');printed.export(path)
 exports.append(dict(part=n,file=str(path.relative_to(O)),print_up_axis='Y',sign=sign,bed_plane_mm=float(t.bounds[0 if sign>0 else 1,1]),assembly_to_print_rotation=rot.tolist(),print_translation_mm=shift.tolist(),other_unsupported_area_mm2=float(printed.area_faces[bad].sum()),watertight=bool(printed.is_watertight),connected_solids=len(printed.split())))

def surface(name,axis,value,cross_lo,cross_hi):
 t=meshes[name];sign=orient[name];bed=t.bounds[0 if sign>0 else 1,1];cross=[i for i in range(3) if i!=axis]
 tri=t.triangles;mask=np.max(abs(tri[:,:,axis]-value),axis=1)<.001
 for k,lo,hi in zip(cross,cross_lo,cross_hi):mask&=(tri[:,:,k].max(1)>lo+.0001)&(tri[:,:,k].min(1)<hi-.0001)
 bad=(t.face_normals[:,1]*sign<-.70712)&((t.triangles_center[:,1]-bed)*sign>.001)
 return dict(part=name,axis='XYZ'[axis],plane_mm=value,matching_face_area_mm2=float(t.area_faces[mask].sum()),support_contact_area_mm2=float(t.area_faces[mask&bad].sum()))
rows=[]
for e in G:
 x,y,z=e['guide_centre_mm'];h=e['guide_half_length_mm'];stroke=9.375 if e['control']=='clock' else 3.756;rod=rod_names[e['module'],e['control']]
 running=[]
 for axis,value,lo,hi in [(0,x-3,[y,z-h-stroke],[y+6,z+h+stroke]),(0,x+3,[y,z-h-stroke],[y+6,z+h+stroke]),(1,y,[x-3,z-h-stroke],[x+3,z+h+stroke]),(1,y+6,[x-3,z-h-stroke],[x+3,z+h+stroke])]:
  r=surface(rod,axis,value,lo,hi);r['pass_check']=r['matching_face_area_mm2']>1 and r['support_contact_area_mm2']<.001;running.append(r)
 for axis,value,lo,hi in [(0,x-3.4,[y-.4,z-h],[y+6.4,z+h]),(0,x+3.4,[y-.4,z-h],[y+6.4,z+h]),(1,y-.6,[x-3.4,z-h],[x+3.4,z+h]),(1,y+6.4,[x-3.4,z-h],[x+3.4,z+h])]:
  rr=[surface(n,axis,value,lo,hi) for n in [e['part'],e['host']]]
  running.append(dict(guide_faces=rr,pass_check=sum(r['matching_face_area_mm2'] for r in rr)>1 and sum(r['support_contact_area_mm2'] for r in rr)<.001))
 rows.append(dict(guide=e['part'],rod=rod,stroke_half_mm=stroke,faces=running,pass_check=all(r['pass_check'] for r in running)))
# The pickup has a pin-following slot as well as its rod connection. Screen all
# of it, rather than exempting small faces near that slot.
pickup_export=next(e for e in exports if e['part']==pickup)
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),scope=__doc__,parts=exports,interfaces=rows,pickup_support_free=pickup_export['other_unsupported_area_mm2']<.001,sliding_surfaces_print_pass=all(e['watertight'] and e['connected_solids']==1 for e in exports) and all(r['pass_check'] for r in rows) and pickup_export['other_unsupported_area_mm2']<.001,limitations=['Geometric face-orientation check; use these exact print orientations and keep slicer supports off all running faces.','Other non-running fixture overhangs and pin-seat shoulders may need local support.','Does not establish printer tolerances, surface finish or friction under load.'])
(O/'Sliding surface print checks.json').write_text(json.dumps(report,indent=2));print('Sliding surfaces',report['sliding_surfaces_print_pass']);print('Failed interfaces',[r for r in rows if not r['pass_check']]);print('Pickup unsupported area',pickup_export['other_unsupported_area_mm2'])
