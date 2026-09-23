"""Use two genuine 5L axles per 26287 clutch sleeve; respect its 8 mm centre web.
Replace the filled coupling placeholder with native 59443 geometry.
"""
from pathlib import Path
import json,numpy as np
from ldraw_mesh import LDraw
O=Path(__file__).resolve().parents[2]/'work/integrated-cam-development';hs=json.loads((O/'hardware.json').read_text());v=np.load(O/'hardware.npz')['vertices'];arr={p['id']:v[p['offset']//3:p['offset']//3+p['vertices']].copy() for p in hs};WX=json.loads((O/'Integrated cam parameters.json').read_text())['write_x'];lib=Path('/Applications/Studio 2.0/ldraw/parts')
ax,_=LDraw(lib/'32073.dat').mesh();ax=ax.reshape(-1,3)*.4
assert np.allclose(np.ptp(ax,axis=0),[40,4.8,4.8])
new=[]
for p in hs:
 n=p['id']
 if n in ['Memory — O-shaft','Write — O-shaft']:
  x=0 if p['bank']=='Memory' else WX;z=0 if p['bank']=='Memory' else 16
  for side,sg in [('left',-1),('right',1)]:
   ident=p['bank']+' — O-'+side+' 5L axle';arr[ident]=ax+[x+sg*24.2,10.2,z];new.append(dict(p,id=ident,source='O-shaft',lego_part='32073',geometry_note='Native 5L axle; terminates 0.2 mm before 26287 central web at |local X|=4 mm.'))
 elif n=='2L LEGO axle joiner — envelope only':
  a,_=LDraw(lib/'59443.dat').mesh();a=a.reshape(-1,3)[:,[2,0,1]]*.4+[-40.4,10.2,16];ident='2L LEGO axle joiner 59443';arr[ident]=a;new.append(dict(p,id=ident,kind='native',lego_part='59443',geometry_note='Native inline smooth 2L joiner with centre divider; not a filled envelope.'))
 else:
  if n.endswith('L097'):p['lego_part']='26287'
  new.append(p)
A=[]
for p in new:p.update(offset=sum(x.size for x in A),vertices=len(arr[p['id']]));A.append(arr[p['id']])
np.savez_compressed(O/'hardware.npz',vertices=np.concatenate(A));(O/'hardware.json').write_text(json.dumps(new,indent=2))
r=dict(clutch_sleeve='26287',central_web_X_mm=[-4,4],axle_inner_ends_local_X_mm=[-4.2,4.2],web_end_clearance_mm=.2,axle_part='32073',axles_per_sleeve=2,memory_output_end_X_mm=44.2,write_core_X_mm=WX,write_output_right_end_X_mm=WX+44.2,memory_worm_left_end_X_mm=-40,coupler='59443',coupler_center_X_mm=-40.4,coupler_divider_thickness_mm=.2,axle_end_gap_in_coupler_mm=.8,limits='Nominal native geometry and end placement. Cross-bore orientation and rotating external clearance checked separately. No torque certification.')
(O/'Clutch axle stop correction.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
