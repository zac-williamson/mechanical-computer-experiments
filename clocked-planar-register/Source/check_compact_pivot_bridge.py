"""Variable-section elastic bending screen of the pin-mounted rear pivot bridge.

Simply supported at its end faces; 120 N assigns the entire clock pivot
reaction to this bridge. Pin flexibility, offset-pin torsion, local contact,
creep, fatigue and print defects are not represented. No material allowable
is assumed and this is not a strength qualification.
"""
import numpy as np,trimesh,json,hashlib
from pathlib import Path
from shapely.geometry import LineString
from shapely.ops import unary_union,polygonize
from shapely.geometry.polygon import orient
R=Path(__file__).resolve().parents[1]/'Compact layout';ps=json.loads((R/'parts.json').read_text());v=np.load(R/'geometry.npz')['vertices'].reshape(-1,3);p=next(p for p in ps if p['id']=='Clock rear pivot bearing bridge');a=v[p['offset']//3:p['offset']//3+p['vertices']];t=trimesh.Trimesh(a,np.arange(len(a)).reshape(-1,3),process=True)
rows=[];L=31.6;F=120.;compliance=[]
for x in np.linspace(24.23,55.77,240):
 lines=trimesh.intersections.mesh_plane(t,[1,0,0],[x,0,0]);polys=list(polygonize(unary_union([LineString(np.round(l[:,1:],7)) for l in lines])))
 polys=[p for p in polys if t.contains([[x,*p.representative_point().coords[0]]])[0]]
 area=sum(p.area for p in polys);cy=sum(p.area*p.centroid.x for p in polys)/area;I=0;c=0
 for p in polys:
  p=orient(p,1)
  for ring in [p.exterior,*p.interiors]:
   a=np.array(ring.coords);a[:,0]-=cy;b=np.roll(a,-1,axis=0);cross=a[:,0]*b[:,1]-b[:,0]*a[:,1];I+=np.sum((a[:,0]**2+a[:,0]*b[:,0]+b[:,0]**2)*cross)/12;c=max(c,abs(a[:,0]).max())
 moment=F/2*(x-24.2)-F*max(x-40,0)
 rows.append([float(x),float(area),float(I),float(c),float(abs(moment)*c/I)])
 compliance.append((moment/F)**2/(1890.*I))
peak=max(rows,key=lambda r:r[-1])
report=dict(scope=__doc__,geometry_sha256=hashlib.sha256((R/'geometry.npz').read_bytes()).hexdigest(),load_N=F,axle_torque_case_Nm=.1,section_samples=len(rows),maximum_nominal_bending_MPa=peak[-1],critical_X_mm=peak[0],assumed_E_MPa=1890,ideal_central_deflection_mm=F*float(np.trapezoid(compliance,[r[0] for r in rows])),mechanically_qualified=False)
(R/'Pivot bridge load screening.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
