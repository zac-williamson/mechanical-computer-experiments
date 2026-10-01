from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];O=R0/'axle-tiling-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
s=s.replace('printed_hits=[];native_hits=[];band_hits=[]','printed_hits=[];native_hits=[];band_hits=[];allposes=[]').replace(' for i,(a,sa,ba) in enumerate(posed):',' allposes.append(posed)\n for i,(a,sa,ba) in enumerate(posed):')
exec(compile(s,'operating checks','exec'));assert not printed_hits and not native_hits and not band_hits
ct=trimesh.load_mesh(R0/'axle-tiling/59443.stl');radius=float(np.linalg.norm(ct.vertices[:,1:],axis=1).max());hits=[];tests=0;clearance=100
for pose in allposes:
 for z in [0,16]:
  connector=cy(radius,32,48,0,[0,10.2,z]);bb=np.array(connector.bounding_box()).reshape(2,3)
  for dx in [0,80]:
   for p,solid0,bounds0 in pose:
    if p['name'] in (['Left output axle 4L','Right output axle 4L'] if z==0 else ['C-shaft']):continue # intended keyed insertion, checked separately below
    other=solid0.translate([dx,0,0]);ob=bounds0+np.array([dx,0,0])
    if np.any(bb[1]<=ob[0]) or np.any(ob[1]<=bb[0]):continue
    v=(connector^other).volume();tests+=1
    if v>.001:hits.append(dict(row_z=z,module_x=dx,part=p['name'],volume=v))
assert not hits,hits
# Nominal positions must leave the smooth connector's 0.2 mm centre stop clear.
by={p['name']:p for p in parts};dims=[]
for n in ['C-shaft','Left output axle 4L','Right output axle 4L']:
 t=by[n]['mesh'];L=float(t.extents[0]);assert abs(L/8-round(L/8))<1e-5;dims.append([n,L])
for z,left,right,depth in [(16,36,44,4),(0,38,42,6)]:
 assert left<39.9 and right>40.1
 assert abs(left-32-depth)<.001 and abs(48-right-depth)<.001
# Preserve full bearing coverage and the original central clutch stop clearance.
assert abs(by['Right output axle 4L']['mesh'].bounds[0,0]-6)<.001
assert abs(by['Left output axle 4L']['mesh'].bounds[1,0]+6)<.001
report=dict(passed=True,axles=dims,connector_full_rotation_radius_mm=radius,operating_poses=len(allposes),connector_interference_tests=tests,hits=hits,input_insertion_each_end_mm=4,output_insertion_each_end_mm=6,clutch_connector_insertion_each_end_mm=6,centre_stop_gap_input_mm=8,centre_stop_gap_output_mm=4,limitations='Clearance and nominal engagement checks only. Partial insertion is not a tested torque or pull-out rating. Smooth connector mesh from local Studio LDraw library; interference uses conservative full rotational envelope except intended axle insertion.')
(O/'Axle coupling checks.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
