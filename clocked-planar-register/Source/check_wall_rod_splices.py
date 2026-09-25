"""Audit positive rod splices: pin engagement, collars and two-rod shear path."""
from pathlib import Path
import json,hashlib,numpy as np,trimesh,manifold3d as m
R=Path(__file__).resolve().parents[1];O=R/'Wall register';P=json.loads((O/'parts.json').read_text());V=np.load(O/'geometry.npz')['vertices'].reshape(-1,3);lookup={p['id']:p for p in P}
def shape(n):
 p=lookup[n];a=V[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True);return m.Manifold(m.Mesh64(t.vertices.astype(float),t.faces.astype(np.uint64)))
def cyl(r,lo,hi,c):return m.Manifold.cylinder(hi-lo,r,circular_segments=32).rotate([-90,0,0]).translate([c[0],lo,c[2]])
records=[]
for key,x,y in [('clock',-116,10),('write',-128,26)]:
 bridge=shape(key+' pinned rod splice bridge');rows=[]
 for idx,z,rod in [(1,-55,'Control '+key.upper()+' direct rod and pickup'),(2,-41,'bit '+key+' vertical control rod')]:
  c=[x,y-.2,z];pin=lookup[key+' rod coupler friction pin '+str(idx)];ss=shape(rod)
  front=cyl(3.4,y-7.7,y-1.7,c)-cyl(2.7,y-7.8,y-1.6,c)
  rear=cyl(3.4,y+1.3,y+7.3,c)-cyl(2.7,y+1.2,y+7.4,c)
  fractions=[(front^bridge).volume()/front.volume(),(rear^ss).volume()/rear.volume()]
  occupied=(cyl(2.4,y-8,y+7.6,c)^(bridge+ss)).volume()
  rows.append(dict(pin=pin['id'],rod=rod,socket_material_fractions=fractions,blocked_bore_mm3=occupied,pass_check=min(fractions)>.97 and occupied<.001 and pin.get('lego_part')=='2780'))
 records.append(dict(control=key,pins=rows,pin_spacing_mm=14,pass_check=all(x['pass_check'] for x in rows)))
report=dict(geometry_sha256=hashlib.sha256((O/'geometry.npz').read_bytes()).hexdigest(),splices=records,rod_splice_pass=all(x['pass_check'] for x in records),limitations=['Nominal socket geometry and shear load path; physical pin grip, fatigue and clearance require printing.','This does not validate the existing closed-guide assembly sequence.'])
(O/'Rod splice checks.json').write_text(json.dumps(report,indent=2));print(report)
