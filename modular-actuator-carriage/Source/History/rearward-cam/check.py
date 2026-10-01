from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];O=R0/'rearward-cam-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
exec(compile(s,'operating checks','exec'));assert not printed_hits and not native_hits and not band_hits
by={p['name']:p for p in parts};old=solid(trimesh.load_mesh(O/'baseline/Lock control rod.stl'));new=by['Lock control rod']['s'];assert (old-new).volume()<.001
# XZ cross-sections are identical through the old and extended cam footprint.
newsection=new^box([-12.5,16,39.9],[12.5,16.1,54.1]);oldsection=(old^box([-12.5,10,39.9],[12.5,10.1,54.1])).translate([0,6,0]);delta=((newsection-oldsection)+(oldsection-newsection)).volume();assert delta<.001
assert np.allclose(by['Bolt follower front bush']['mesh'].bounds[:,1],[15.6,19.6],atol=.001)
# Follower remains on the original 3L axle, without reaching its rear retainer.
assert by['Bolt follower axle 3L']['mesh'].bounds[0,1]<15.6
assert by['Bolt follower axle 3L']['mesh'].bounds[1,1]>19.6
# Verify a continuous bed footprint for the added central plate.
extra=new-old;bed_column=(extra^box([-13,14.2,40],[13,19.61,40.2]));assert bed_column.volume()>5
# Strict new-material tests at 39 release positions, including native hardware.
hits=[];contact_comparison=[]
for lift in np.linspace(0,3.8,39):
 lo=0.;hi=8.25
 for _ in range(60):
  cx=(lo+hi)/2
  if math.sqrt(144-(3.75-cx)**2)-math.sqrt(144-(-3.75-cx)**2)<3.8:lo=cx
  else:hi=cx
 cx=(lo+hi)/2;cz=47.6+math.sqrt(144-(3.75-cx)**2);x=math.sqrt(144-(cz-47.6-float(lift))**2)-cx
 a=extra.translate([x,0,0]);bb=np.array(a.bounding_box()).reshape(2,3)
 for p in parts:
  if p['name']=='Lock control rod' or p['kind']=='elastic':continue
  other=p['s'].translate([0,0,float(lift)]) if p['motion']=='bolt' else p['s'];ob=np.array(other.bounding_box()).reshape(2,3)
  if np.any(bb[1]<=ob[0]) or np.any(ob[1]<=bb[0]):continue
  v=(a^other).volume()
  if p['name']=='Bolt follower front bush':
   old_contact=(old.translate([x,0,0])^other.translate([0,-4,0])).volume()
   # The cylinder proxy intentionally bears on the inherited faceted cam.
   # Confirm the contact area per unit Y width is unchanged, not a new clash.
   error=abs(v/4-old_contact/2.6);assert error<.001,(lift,v,old_contact,error)
   contact_comparison.append(dict(lift=float(lift),normalised_contact_difference_mm2=error))
   continue
  if v>.01:hits.append([float(lift),p['name'],v])
assert not hits,hits
(O/'Cam extension checks.json').write_text(json.dumps(dict(passed=True,profile_difference_mm3=delta,extra_material_interferences=hits,release_samples=39,matched_inherited_cam_contact=contact_comparison,nominal_guide_gap_Y_mm=.4,old_follower_centre_Y=13.6,new_follower_centre_Y=17.6,guide_centre_Y=27.6,nominal_lever_arm_before_mm=14,nominal_lever_arm_after_mm=10,limits='Centre-based moment-arm estimate only; actual contact distribution, friction and elastic forces are not simulated.'),indent=2));print('Cam extension checks passed',flush=True)
