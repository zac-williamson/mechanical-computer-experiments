from pathlib import Path
import json,numpy as np,trimesh,manifold3d as m
from shapely.geometry import LineString
R=Path(__file__).resolve().parents[1];O=R/'Planar register';s=(R/'Source/full_contact_inventory.py').read_text();exec(s[:s.index('records={}')])
P={p['id']:trimesh.load(O/(p['id']+'.stl')) for p in ps};S={n:m.Manifold(m.Mesh64(np.array(t.vertices,copy=True),np.array(t.faces,dtype=np.uint64,copy=True))) for n,t in P.items()};hits={}
for bank in ['Memory','Write']:
 shift=np.array([0,0,0]) if bank=='Memory' else np.array([96.8,0,16])
 for q in np.linspace(-4.6,4.6,93):
  f=min(trace,key=lambda f:abs(f['q']-q));T=trimesh.transformations.rotation_matrix(np.radians(f['b']),[0,1,0],[13.192323604,10.2,32.128448698]);a=np.array([37.192323604,10.2,28.128448698])+shift;b=trimesh.transform_points([[22.192323604,10.2,30.628448698]],T)[0]+shift
  line=LineString([a[[0,2]],b[[0,2]]]);poly=line.buffer(3.2,quad_segs=24).difference(line.buffer(2,quad_segs=24));cs=m.CrossSection([np.asarray(poly.exterior.coords)[:-1]]+[np.asarray(r.coords)[:-1] for r in poly.interiors],m.FillRule.EvenOdd);band=cs.extrude(1.2).transform([[1,0,0,0],[0,0,1,9.6],[0,1,0,0]])
  for other in [-3.75,3.75]:
   qm,qe=(q,other) if bank=='Memory' else (other,q)
   for p in ps:
    vol=max(0,float((band^S[p['id']].transform(pose(p,qm,qe)[:3,:])).volume()))
    if vol>.005 and vol>hits.get((bank,p['id']),{}).get('volume_mm3',0):hits[bank,p['id']]=dict(band=bank,part=p['id'],q=q,other=other,volume_mm3=vol)
report=dict(samples_per_band=186,printed_contacts=list(hits.values()),scope='Source capsule envelopes: Y9.6..10.8, outer radius3.2, inner radius2.0. Not a force-dependent band shape or native-hardware collision proof.');(O/'Actuator band checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
