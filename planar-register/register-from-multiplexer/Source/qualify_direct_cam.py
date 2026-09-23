from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from shapely.geometry import LineString,Polygon,Point
from direct_cam_math import cam_lift,bolt_lift,CREST,INTERCEPT
R=Path(__file__).resolve().parents[1];O=R/'Planar register';s=(R/'Source/full_contact_inventory.py').read_text();exec(s[:s.index('records={}')])
parts={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in ps};solids={n:m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True))) for n,t in parts.items()}
def band(up):
 line=LineString([(18.6,49.3),(18.6,57.3+up)]);poly=line.buffer(3.1,quad_segs=32).difference(line.buffer(2.5,quad_segs=32));cs=m.CrossSection([np.asarray(poly.exterior.coords)[:-1]]+[np.asarray(i.coords)[:-1] for i in poly.interiors],m.FillRule.EvenOdd);return cs.extrude(1).transform([[0,0,1,30.1],[1,0,0,0],[0,1,0,0]])
hits={};lo=np.array([np.inf]*3);hi=-lo
for qe in np.linspace(-4.6,4.6,93):
 for qm in [-4.6,0,4.6]:
  up=bolt_lift(qm,qe);b=band(up)
  for p in ps:
   T=pose(p,qm,qe);ix=b^solids[p['id']].transform(T[:3]);vol=max(0,float(ix.volume()))
   if vol>.005:hits.setdefault(p['id'],dict(part=p['id'],volume_mm3=vol,qm=qm,qe=qe))
   vs=trimesh.transform_points(parts[p['id']].vertices,T);lo=np.minimum(lo,vs.min(0));hi=np.maximum(hi,vs.max(0))
lo=np.minimum(lo,v.min(0));hi=np.maximum(hi,v.max(0))
poly=Polygon([[32,38.5],[78,38.5],[82,40.4],[92.2,40.4],[92.2,62],[82,62],[42,INTERCEPT-1.8*42],[CREST,61.5],[32,61.5]])
distances=[]
for qe in np.linspace(-4.6,4.6,921):
 point=Point(38-qe,57.3+cam_lift(qe));distances.append(point.distance(poly)-3.6)
rows=[]
for q in np.linspace(.2,4.6,89):rows.append(39.9+cam_lift(q-.6)-45.3)
report=dict(timing=dict(combined_input_error_mm=.6,last_possible_D_contact_q=.2,minimum_bolt_gap_before_reconnection_mm=min(rows),passes_0_4mm_gap=min(rows)>=.4,closed_bolt_engagement_mm=5.4,maximum_bolt_lift_mm=cam_lift(4.6),pressure_angle_degrees=float(np.degrees(np.arctan(1.8)))),cam_contact=dict(samples=921,minimum_roller_clearance_mm=min(distances),maximum_gap_mm=max(distances),explanation='Zero clearance denotes rolling contact. At negative overtravel the guide head stop holds the bolt while the cam retreats.'),band=dict(samples=279,printed_contacts=list(hits.values()),closed_loop_path_mm=16+2*np.pi*2.8,max_loop_path_mm=2*(8+cam_lift(4.6))+2*np.pi*2.8,target_force_N=[.5,2],qualification='Band force-extension not measured; nominal capsule excludes force-dependent shape.'),sampled_bounds_mm=[lo.tolist(),hi.tolist()],sampled_envelope_mm=(hi-lo).tolist(),guide_friction_screen=[dict(assumed_mu=mu,band_force_N=2,vertical_contact_force_N=2/(1-1.8*mu),horizontal_force_N=3.6/(1-1.8*mu)) for mu in [0,.1,.2,.3,.4]],limits=['High cam pressure angle: ideal guide-friction self-lock threshold mu=1/1.8, about0.556. Actual friction must be measured; this is not a force-driven qualification.','Prescribed rigid motion. No physical 0.1Nm endurance, jam or loaded-engagement proof.'])
assert not hits,hits
assert min(distances)>-1e-6,min(distances)
assert min(rows)>=.4,min(rows)
(O/'Qualification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
P=O/'Prototype print parts';P.mkdir(exist_ok=True);records=[]
for p in ps:
 t=parts[p['id']].copy();name=p['id']
 if p['motion']=='carriage' and name!='Write — Direct cam plate':normal=[-1,0,0]
 elif name=='Write — Direct cam plate':normal=[0,1,0]
 elif name=='Memory — Front bearing cheek':normal=[0,-1,0]
 elif name=='Direct lock bolt':normal=[0,1,0]
 else:
  candidates=[]
  for ax in range(3):
   for sign in [-1,1]:
    nn=np.eye(3)[ax]*sign;a=t.copy();a.apply_transform(trimesh.geometry.align_vectors(nn,[0,0,-1]));a.apply_translation(-a.bounds[0]);tri=a.triangles;onbed=np.all(abs(tri[:,:,2])<1e-5,axis=1);candidates.append((float(a.area_faces[onbed].sum()),nn))
  normal=max(candidates,key=lambda x:x[0])[1]
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(P/(name+'.stl'),file_type='stl_ascii');records.append(dict(part=name,size_mm=t.extents.tolist(),bed_normal=np.asarray(normal).tolist(),support_review_required=name!='Write — Direct cam plate'))
(P/'Orientations.json').write_text(json.dumps(records,indent=2));print('Exported',len(records),'oriented parts')
