from pathlib import Path
import os,json
R0=Path(__file__).resolve().parents[1];O=R0/'single-band-candidate';os.environ['PLANAR_OUTPUT']=str(O)
s=(R0/'validate.py').read_text().replace("t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)","t=trimesh.load_mesh(R/(p['name']+'.stl')) if p['kind']=='printed' else trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);p=dict(p)")
s=s.replace('printed_hits=[]',"parts=[p for p in parts if p['motion']!='lock-band']\nprinted_hits=[]")
exec(compile(s,'structure checks','exec'));assert not printed_hits and not native_hits and not band_hits
(O/'Structural status.json').write_text(json.dumps(dict(structural_motion_passed=True,lock_band_checked=False,published=False,reason='Awaiting confirmation of the single-band route before changing anchors, wrap length or viewer.'),indent=2))
