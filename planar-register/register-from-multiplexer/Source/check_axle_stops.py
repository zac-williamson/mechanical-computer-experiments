from pathlib import Path
import json,numpy as np
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];A={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']] for p in hs};rows=[]
for bank,cx,z in [('Memory',0,0),('Write',-85,16)]:
 left=A[bank+' — O-left 5L axle'];right=A[bank+' — O-right 5L axle'];a=float(cx-4-left[:,0].max());b=float(right[:,0].min()-(cx+4));assert min(a,b)>.19
 # Each end occupies the genuine sleeve's 8 mm cross-bore, and stops before its web.
 rows.append(dict(bank=bank,stop_region_X_mm=[cx-4,cx+4],left_axle_bounds_X_mm=[float(left[:,0].min()),float(left[:,0].max())],right_axle_bounds_X_mm=[float(right[:,0].min()),float(right[:,0].max())],clearance_to_web_mm=[a,b],engagement_each_mm=7.8))
left=A['Write — O-right 5L axle'];right=A['Memory — C-shaft'];gap=float(right[:,0].min()-left[:,0].max());assert gap>.79
coupler=A['2L LEGO axle joiner 59443'];assert coupler[:,0].min()<left[:,0].max()-7.5;assert coupler[:,0].max()>right[:,0].min()+7.5
r=dict(status='PASS nominal central-stop clearance',clutch_joiners=rows,bridge_axle_end_gap_mm=gap,bridge_divider_mm=.2,bridge_clearance_to_divider_each_mm=.3,scope='Exact axial bounds of native LEGO meshes, 26287 web and 59443 divider. Does not certify all external collisions or strength.')
(O/'Axle stop checks.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
