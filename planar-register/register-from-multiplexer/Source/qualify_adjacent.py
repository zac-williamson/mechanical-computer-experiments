from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from shapely.geometry import LineString
R=Path(__file__).resolve().parents[1];O=R/'Planar register';meta=json.loads((O/'printed-parts.json').read_text());H=json.loads((O/'hardware.json').read_text());vv=np.load(O/'hardware.npz')['vertices']
def angle(q):return np.arctan2(6.8,10)+np.arcsin((q-6.8)/np.sqrt(146.24))
def lift(q):return max(0,float(54+18*np.sin(angle(q))+8*np.cos(angle(q))+3.6-57))
# ±0.6mm is an explicit combined input slot/stack error assumption, not measured printing performance.
rows=[]
for q in np.linspace(-4.6,4.6,185):
 eff=np.clip(np.array([q-.6,q+.6]),-4.6,4.6);ls=[lift(x) for x in eff];rows.append(dict(q=float(q),min_bolt_lift=min(ls),max_bolt_lift=max(ls)))
reconnect=[x for x in rows if x['q']>=.2-1e-8];mingap=min(39.9+x['min_bolt_lift']-45.3 for x in reconnect)
# Band envelope: capsule section in YZ, 1.2mm wide, retained between lug shoulders.
def band(up):
 path=LineString([(34,41.9),(34,59.4+up)]);poly=path.buffer(4.1,quad_segs=32).difference(path.buffer(3.3,quad_segs=32))
 cs=m.CrossSection([np.asarray(poly.exterior.coords)[:-1]],m.FillRule.EvenOdd)
 for h in poly.interiors:cs-=m.CrossSection([np.asarray(h.coords)[:-1]],m.FillRule.EvenOdd)
 raw=cs.extrude(1.2).to_mesh64();v=np.asarray(raw.vert_properties)[:,:3];v=np.c_[24.0+v[:,2],v[:,0],v[:,1]]
 return m.Manifold(m.Mesh64(np.array(v,dtype=np.float64,order="C",copy=True),np.array(raw.tri_verts,dtype=np.uint64,order="C",copy=True)))
parts={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in meta}
solids={n:m.Manifold(m.Mesh64(np.ascontiguousarray(t.vertices),np.ascontiguousarray(t.faces,dtype=np.uint64))) for n,t in parts.items()}
issues=[];lo=np.array([np.inf]*3);hi=-lo
for q in np.linspace(-4.6,4.6,47):
 up=lift(q);b=band(up);assert b.volume()>10, (b.status(),b.volume())
 for qm in [-4.6,4.6]:
  for p in meta:
   T=np.eye(4)
   if p['motion']=='carriage':T[0,3]=qm if p['bank']=='Memory' else q
   if p['motion']=='bolt':T[2,3]=up
   if p['motion']=='release-crank':T=trimesh.transformations.rotation_matrix(angle(q),[0,1,0],[60,0,54])
   a=trimesh.transform_points(parts[p['id']].vertices,T);lo=np.minimum(lo,a.min(0));hi=np.maximum(hi,a.max(0))
   ix=b^solids[p['id']].transform(T[:3,:]);vol=max(0,float(ix.volume()))
   if vol>.005:issues.append(dict(part=p['id'],q=float(q),qm=qm,volume=vol))
# Include the source hardware envelopes; new rotating crank shafts stay within the frame XZ envelope.
lo=np.minimum(lo,vv.min(0));hi=np.maximum(hi,vv.max(0))
F=50;web=7.9-5.2
report=dict(timing=dict(combined_input_error_mm=.6,last_possible_D_contact_q=.2,minimum_bolt_gap_before_reconnection_mm=mingap,passes_0_4mm_gap=mingap>=.4),band=dict(capsule_envelope_collision_count=len(issues),collisions=issues[:20],closed_loop_path_mm=2*(59.4-41.9)+2*np.pi*3.7,max_loop_path_mm=2*(59.4+lift(4.6)-41.9)+2*np.pi*3.7,selection='Measure a band giving about 0.5 N closed and no more than 2 N fully withdrawn. These are design targets, not a qualified off-the-shelf band specification.'),sampled_envelope_mm=(hi-lo).tolist(),sampled_bounds_mm=[lo.tolist(),hi.tolist()],screening=dict(assumed_latch_force_N=F,keeper_web_bending_MPa=1.5*F*5.6/(6.4*web**2),bolt_bending_MPa=1.5*F*5.6/(4.8*3.9**2),qualification='50 N is a screening load, not a proven bound derived from every 0.1 Nm axle condition. No physical torque qualification. Stress concentrations, creep and layer adhesion are unmodelled.'),remaining_physical_tests=['Verify loaded bolt withdrawal and clutch disengagement sequence.','Measure band force-extension and end-state stability while WRITE remains active.','Apply 0.1 Nm in both directions to every driven axle, including interrupted writes and deliberately blocked conditions.','Slicer/support-removal and printed fits must be checked before assembling all parts.'])
(O/'Qualification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
# Prototype orientation candidates. No hidden support-free claim.
P=O/'Prototype print parts';P.mkdir(exist_ok=True);records=[]
for p in meta:
 t=parts[p['id']].copy()
 if p['motion']=='carriage' and p['id']!='Write — Flat pin-mounted link':normal=[-1,0,0]
 elif p['id']=='Write — Flat pin-mounted link':normal=[0,1,0]
 elif p['id']=='Adjacent release crank':normal=[0,1,0]
 elif p['id']=='Adjacent lock bolt':normal=[0,1,0]
 else:
  candidates=[]
  for ax in range(3):
   for sign in [-1,1]:
    n=np.eye(3)[ax]*sign;a=t.copy();a.apply_transform(trimesh.geometry.align_vectors(n,[0,0,-1]));a.apply_translation(-a.bounds[0]);tri=a.triangles;bt=tri[np.all(abs(tri[:,:,2])<1e-4,axis=1)];ar=np.linalg.norm(np.cross(bt[:,1]-bt[:,0],bt[:,2]-bt[:,0]),axis=1).sum()/2;candidates.append((ar,n))
  normal=max(candidates,key=lambda a:a[0])[1]
 t.apply_transform(trimesh.geometry.align_vectors(normal,[0,0,-1]));t.apply_translation(-t.bounds[0]);t.export(P/(p['id']+'.stl'),file_type='stl_ascii');records.append(dict(part=p['id'],size_mm=t.extents.tolist(),bed_normal=np.asarray(normal).tolist(),support_review_required=True))
(P/'Orientations.json').write_text(json.dumps(records,indent=2));print('Exported',len(records),'oriented prototype parts')
